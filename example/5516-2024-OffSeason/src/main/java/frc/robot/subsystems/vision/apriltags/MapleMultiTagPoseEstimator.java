package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;
import frc.robot.constants.RobotMode;
import frc.robot.utils.CustomMaths.Statistics;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static frc.robot.constants.VisionConstants.*;
import static frc.robot.constants.LogPaths.APRIL_TAGS_VISION_PATH;

public class MapleMultiTagPoseEstimator {
    public static final class RobotPoseEstimationResult {
        public final Pose2d pointEstimation;
        public final Matrix<N3, N1> estimationStandardError;
        public final double translationXStandardDeviationMeters, translationYStandardDeviationMeters, rotationalStandardDeviationRadians;

        public RobotPoseEstimationResult(Pose2d pointEstimation, double translationXStandardDeviationMeters, double translationYStandardDeviationMeters, double rotationalStandardDeviationRadians) {
            this.pointEstimation = pointEstimation;
            this.estimationStandardError = VecBuilder.fill(translationXStandardDeviationMeters, translationYStandardDeviationMeters, rotationalStandardDeviationRadians);
            this.translationXStandardDeviationMeters = translationXStandardDeviationMeters;
            this.translationYStandardDeviationMeters = translationYStandardDeviationMeters;
            this.rotationalStandardDeviationRadians = rotationalStandardDeviationRadians;
        }
    }

    public static final boolean LOG_DETAILED_FILTERING_DATA = Robot.CURRENT_ROBOT_MODE != RobotMode.REAL;

    private final AprilTagFieldLayout fieldLayout;
    private final VisionResultsFilter filter;
    private final List<PhotonCameraProperties> camerasProperties;
    public MapleMultiTagPoseEstimator(AprilTagFieldLayout aprilTagFieldLayout, VisionResultsFilter filter, List<PhotonCameraProperties> camerasProperties) {
        this.fieldLayout = aprilTagFieldLayout;
        this.filter = filter;
        this.camerasProperties = camerasProperties;
    }

    final List<Pose3d> robotPose3dObservations = new ArrayList<>(),
            observedAprilTagsPoses = new ArrayList<>(),
            observedVisionTargetPoseInFieldLayout = new ArrayList<>();
    private void fetchRobotPose3dEstimationsFromCameraInputs(AprilTagVisionIO.CameraInputs[] cameraInputs, Pose2d currentOdometryPose) {
        robotPose3dObservations.clear(); observedAprilTagsPoses.clear(); observedVisionTargetPoseInFieldLayout.clear();
        if (cameraInputs.length != camerasProperties.size())
            throw new IllegalArgumentException("camera inputs length " + cameraInputs.length + " does not match camera properties size " + camerasProperties.size());

        for (int i = 0; i < cameraInputs.length; i++)
            fetchSingleCameraInputs(cameraInputs[i], camerasProperties.get(i), currentOdometryPose);
    }

    private void fetchSingleCameraInputs(AprilTagVisionIO.CameraInputs cameraInput, PhotonCameraProperties cameraProperty, Pose2d currentOdometryPose) {
        calculateRobotPose3dFromSingleObservation(cameraInput.bestCameraToField, cameraProperty.robotToCamera)
                .ifPresent(robotPose3dObservations::add);
        if (!LOG_DETAILED_FILTERING_DATA) return;
        for (int i = 0; i < cameraInput.fiducialMarksID.length; i++) {
            if (cameraInput.fiducialMarksID[i] != -1) {
                fieldLayout.getTagPose(cameraInput.fiducialMarksID[i])
                        .ifPresent(observedVisionTargetPoseInFieldLayout::add);
                observedAprilTagsPoses.add(calculateObservedAprilTagTargetPose(
                        cameraInput.bestCameraToTargets[i],
                        cameraProperty.robotToCamera,
                        currentOdometryPose
                ));
            }
        }
    }

    private Pose3d calculateObservedAprilTagTargetPose(Transform3d bestCameraToTarget, Transform3d robotToCamera, Pose2d currentOdometryPose) {
        return new Pose3d(currentOdometryPose).transformBy(robotToCamera).transformBy(bestCameraToTarget);
    }
    private Optional<Pose3d> calculateRobotPose3dFromSingleObservation(Optional<Transform3d> bestCameraToField, Transform3d robotToCamera) {
        return bestCameraToField.map(cameraToField -> new Pose3d()
                .transformBy(cameraToField)
                .transformBy(robotToCamera.inverse())
        );
    }

    final List<Pose3d> validRobotPoseEstimations = new ArrayList<>();
    final List<Pose3d> invalidRobotPoseEstimations = new ArrayList<>();
    private void applyFilteringToRawRobotPose3dEstimations() {
        validRobotPoseEstimations.clear();
        invalidRobotPoseEstimations.clear();
        for (final Pose3d estimation : robotPose3dObservations) {
            if (filter.isResultValid(estimation))
                validRobotPoseEstimations.add(estimation);
            else
                invalidRobotPoseEstimations.add(estimation);
        }
    }

    /**
     * using the filtering mechanism, find out the best guess of the robot pose and the standard error
     * @param cameraInputs the inputs of the cameras
     * @return (optionally) the best guess of the robot pose and the standard error, if there are valid targets
     * */
    public Optional<RobotPoseEstimationResult> estimateRobotPose(AprilTagVisionIO.CameraInputs[] cameraInputs, Pose2d currentOdometryPose) {
        if (cameraInputs.length != camerasProperties.size())
            throw new IllegalStateException("camera inputs length" + cameraInputs.length
                    + " does not match cameras properties length: " + camerasProperties.size());

        fetchRobotPose3dEstimationsFromCameraInputs(cameraInputs, currentOdometryPose);

        applyFilteringToRawRobotPose3dEstimations();

        if (Robot.CURRENT_ROBOT_MODE != RobotMode.REAL)
            logFilteringData();

        return getEstimationResultFromValidObservations();
    }

    private Optional<RobotPoseEstimationResult> getEstimationResultFromValidObservations() {
        final int n = validRobotPoseEstimations.size();
        if (n == 0 || observedVisionTargetPoseInFieldLayout.size() < MINIMUM_TAGS_NUM)
            return Optional.empty();
        if (n == 1)
            return Optional.of(new RobotPoseEstimationResult(
                    validRobotPoseEstimations.get(0).toPose2d(),
                    TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION,
                    TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION,
                    ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION
            ));

        final double[]
                robotPoseEstimationsXMeters = validRobotPoseEstimations.stream().mapToDouble(Pose3d::getX).toArray(),
                robotPoseEstimationsYMeters = validRobotPoseEstimations.stream().mapToDouble(Pose3d::getY).toArray(),
                robotPoseEstimatorThetaRadians = validRobotPoseEstimations.stream().mapToDouble(pose3d -> pose3d.toPose2d().getRotation().getRadians()).toArray();

        final Translation2d translationPointEstimate = new Translation2d(
                Statistics.getMean(robotPoseEstimationsXMeters),
                Statistics.getMean(robotPoseEstimationsYMeters)
        );
        final Rotation2d rotationPointEstimate = Rotation2d.fromRadians(
                Statistics.getMean(robotPoseEstimatorThetaRadians)
        );

        double estimationStandardDeviationX = Statistics.getStandardDeviation(robotPoseEstimationsXMeters),
                estimationStandardDeviationY = Statistics.getStandardDeviation(robotPoseEstimationsYMeters),
                estimationStandardDeviationTheta = Statistics.getStandardDeviation(robotPoseEstimatorThetaRadians);

        return Optional.of(new RobotPoseEstimationResult(
                new Pose2d(translationPointEstimate, rotationPointEstimate),
                estimationStandardDeviationX,
                estimationStandardDeviationY,
                estimationStandardDeviationTheta
        ));
    }

    /**
     * Log the filtering data
     * */
    private void logFilteringData() {
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/CurrentFilterImplementation", filter.getFilterImplementationName());
        if (!LOG_DETAILED_FILTERING_DATA) return;

        /* these are the detailed filtering data, logging them on RobotRIO1.0 is a bad idea, if you want them, replay the log */
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/ValidPoseEstimations", validRobotPoseEstimations.toArray(Pose3d[]::new));
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/InvalidPoseEstimations", invalidRobotPoseEstimations.toArray(Pose3d[]::new));
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/VisibleFieldTargets", observedVisionTargetPoseInFieldLayout.toArray(Pose3d[]::new));
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/AprilTagsObservedPositions/", observedAprilTagsPoses.toArray(Pose3d[]::new));
    }
}

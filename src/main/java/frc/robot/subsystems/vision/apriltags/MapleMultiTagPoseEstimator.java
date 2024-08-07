package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Config.PhotonCameraProperties;
import frc.robot.utils.MapleMaths.Statistics;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static frc.robot.Constants.VisionConfigs.*;
import static frc.robot.Constants.LogConfigs.APRIL_TAGS_VISION_PATH;

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

    public static final boolean LOG_DETAILED_FILTERING_DATA = Robot.CURRENT_ROBOT_MODE != Constants.RobotMode.REAL;

    private final AprilTagFieldLayout fieldLayout;
    private final VisionResultsFilter filter;
    private final List<PhotonCameraProperties> camerasProperties;
    public MapleMultiTagPoseEstimator(AprilTagFieldLayout aprilTagFieldLayout, VisionResultsFilter filter, List<PhotonCameraProperties> camerasProperties) {
        this.fieldLayout = aprilTagFieldLayout;
        this.filter = filter;
        this.camerasProperties = camerasProperties;
    }

    final List<Integer> observedAprilTagsIDs = new ArrayList<>();
    final List<Pose3d> robotPose3dObservations = new ArrayList<>(),
            observedAprilTagsPoses = new ArrayList<>(),
            observedVisionTargetPoseInFieldLayout = new ArrayList<>();
    private void fetchRobotPose3dEstimationsFromCameraInputs(AprilTagVisionIO.CameraInputs[] cameraInputs, Pose2d currentOdometryPose) {
        observedAprilTagsIDs.clear(); robotPose3dObservations.clear(); observedAprilTagsPoses.clear(); observedVisionTargetPoseInFieldLayout.clear();
        for (int cameraID = 0; cameraID < cameraInputs.length; cameraID++)
            for (int i = 0; i < cameraInputs[cameraID].currentTargetsCount; i++)
                fetchRobotPose3dEstimationFromSingleTarget(
                        camerasProperties.get(cameraID).robotToCamera,
                        cameraInputs[cameraID].bestCameraToTargets[i],
                        cameraInputs[cameraID].fiducialMarksID[i],
                        currentOdometryPose
                );
    }

    private void fetchRobotPose3dEstimationFromSingleTarget(Transform3d robotToCamera, Transform3d bestCameraToTarget, int fiducialMarkID, Pose2d currentOdometryPose) {
        calculateRobotPose3dFromSingleObservation(
                fiducialMarkID, bestCameraToTarget, robotToCamera
        ).ifPresent(robotPose3dObservation -> {
            robotPose3dObservations.add(robotPose3dObservation);
            observedAprilTagsIDs.add(fiducialMarkID);
            if (LOG_DETAILED_FILTERING_DATA) observedAprilTagsPoses.add(calculateObservedAprilTagTargetPose(bestCameraToTarget, robotToCamera, currentOdometryPose));
            if (LOG_DETAILED_FILTERING_DATA) observedVisionTargetPoseInFieldLayout.add(fieldLayout.getTagPose(fiducialMarkID).orElse(new Pose3d()));
        });
    }

    final List<Pose2d> filteredRobotPoseEstimations = new ArrayList<>();
    private void applyFilteringToRawRobotPose3dEstimations() {
        filteredRobotPoseEstimations.clear();
        for (Pose3d pose3d:robotPose3dObservations)
            if (filter.test(pose3d))
                filteredRobotPoseEstimations.add(pose3d.toPose2d());
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

        if (Robot.CURRENT_ROBOT_MODE != Constants.RobotMode.REAL)
            logFilteringData();

        return getEstimationResultFromValidObservations();
    }

    private Pose3d calculateObservedAprilTagTargetPose(Transform3d bestCameraToTarget, Transform3d robotToCamera, Pose2d currentOdometryPose) {
        return new Pose3d(currentOdometryPose).transformBy(robotToCamera).transformBy(bestCameraToTarget);
    }
    private Optional<Pose3d> calculateRobotPose3dFromSingleObservation(int fiducialID, Transform3d bestCameraToTarget, Transform3d robotToCamera) {
        final Optional<Pose3d> targetPosition = fieldLayout.getTagPose(fiducialID);
        return targetPosition.map(tagAbsolutePoseOnField ->
                tagAbsolutePoseOnField
                        .transformBy(bestCameraToTarget.inverse())
                        .transformBy(robotToCamera.inverse())
        );
    }

    private Optional<RobotPoseEstimationResult> getEstimationResultFromValidObservations() {
        final int n = filteredRobotPoseEstimations.size();
        if (n == 0)
            return Optional.empty();
        if (n == 1)
            return Optional.of(new RobotPoseEstimationResult(
                    filteredRobotPoseEstimations.get(0),
                    TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION,
                    TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION,
                    ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION
            ));

        final double[]
                robotPoseEstimationsXMeters = filteredRobotPoseEstimations.stream().mapToDouble(Pose2d::getX).toArray(),
                robotPoseEstimationsYMeters = filteredRobotPoseEstimations.stream().mapToDouble(Pose2d::getY).toArray(),
                robotPoseEstimatorThetaRadians = filteredRobotPoseEstimations.stream().mapToDouble(pose2d -> pose2d.getRotation().getRadians()).toArray();

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
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/RobotPose2dObservationsFiltered", filteredRobotPoseEstimations.toArray(Pose2d[]::new));
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/RobotPose3dsResults", robotPose3dObservations.toArray(Pose3d[]::new));
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/VisibleFieldTargets", observedVisionTargetPoseInFieldLayout.toArray(Pose3d[]::new));
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/AprilTagsObservedPositions/AprilTag ID", observedAprilTagsIDs.stream().mapToInt(i -> i).toArray());
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/AprilTagsObservedPositions/", observedAprilTagsPoses.toArray(Pose3d[]::new));
    }
}

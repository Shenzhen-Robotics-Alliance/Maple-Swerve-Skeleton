package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.CustomMaths.Statistics;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static frc.robot.constants.VisionConstants.*;
import static frc.robot.constants.LogPaths.APRIL_TAGS_VISION_PATH;

public class MapleMultiTagPoseEstimator {
    public static final class RobotPoseEstimationResult {
        public Pose2d pointEstimation;
        public double translationXStandardDeviationMeters, translationYStandardDeviationMeters, rotationalStandardDeviationRadians;

        public RobotPoseEstimationResult(Pose2d pointEstimation, double translationXStandardDeviationMeters, double translationYStandardDeviationMeters, double rotationalStandardDeviationRadians) {
            this.pointEstimation = pointEstimation;
            this.translationXStandardDeviationMeters = translationXStandardDeviationMeters;
            this.translationYStandardDeviationMeters = translationYStandardDeviationMeters;
            this.rotationalStandardDeviationRadians = rotationalStandardDeviationRadians;
        }

        public Matrix<N3, N1> getEstimationStandardError() {
            return VecBuilder.fill(translationXStandardDeviationMeters, translationYStandardDeviationMeters, rotationalStandardDeviationRadians);
        }
    }

    public static final boolean LOG_DETAILED_FILTERING_DATA = true;
            // Robot.CURRENT_ROBOT_MODE != RobotMode.REAL;

    private final AprilTagFieldLayout fieldLayout;
    private final VisionResultsFilter filter;
    private final List<PhotonCameraProperties> camerasProperties;
    public MapleMultiTagPoseEstimator(AprilTagFieldLayout aprilTagFieldLayout, VisionResultsFilter filter, List<PhotonCameraProperties> camerasProperties) {
        this.fieldLayout = aprilTagFieldLayout;
        this.filter = filter;
        this.camerasProperties = camerasProperties;
    }

    final List<Pose3d>
            robotPose3dObservationsMultiTag = new ArrayList<>(),
            robotPose3dObservationsSingleTag = new ArrayList<>(),
            observedAprilTagsPoses = new ArrayList<>(),
            observedVisionTargetPoseInFieldLayout = new ArrayList<>();
    private void fetchRobotPose3dEstimationsFromCameraInputs(AprilTagVisionIO.CameraInputs[] cameraInputs, Pose2d currentOdometryPose) {
        robotPose3dObservationsMultiTag.clear(); robotPose3dObservationsSingleTag.clear(); observedAprilTagsPoses.clear(); observedVisionTargetPoseInFieldLayout.clear();

        if (cameraInputs.length != camerasProperties.size())
            throw new IllegalArgumentException("camera inputs length " + cameraInputs.length + " does not match camera properties size " + camerasProperties.size());

        for (int i = 0; i < cameraInputs.length; i++)
            fetchSingleCameraInputs(cameraInputs[i], camerasProperties.get(i), currentOdometryPose);
    }

    private void fetchSingleCameraInputs(AprilTagVisionIO.CameraInputs cameraInput, PhotonCameraProperties cameraProperty, Pose2d currentOdometryPose) {
        calculateVisibleTagsPosesForLog(cameraInput, cameraProperty, currentOdometryPose);

        /* if there is multi-solvepnp result, we only trust that */
        Optional<Pose3d> multiSolvePNPPoseEstimation = calculateRobotPose3dFromMultiSolvePNPResult(
                cameraProperty.robotToCamera,
                cameraInput.bestFieldToCamera
        );
        if (multiSolvePNPPoseEstimation.isPresent()) {
            robotPose3dObservationsMultiTag.add(multiSolvePNPPoseEstimation.get());
            return;
        }

        for (int i = 0; i < cameraInput.currentTargetsCount; i++)
            calculateRobotPose3dFromSingleObservation(
                    cameraProperty.robotToCamera,
                    cameraInput.bestCameraToTargets[i],
                    cameraInput.fiducialMarksID[i]
            ).ifPresent(robotPose3dObservationsSingleTag::add);
    }

    private Pose3d calculateObservedAprilTagTargetPose(Transform3d bestCameraToTarget, Transform3d robotToCamera, Pose2d currentOdometryPose) {
        return new Pose3d(currentOdometryPose)
                .transformBy(robotToCamera)
                .transformBy(bestCameraToTarget);
    }

    private Optional<Pose3d> calculateRobotPose3dFromSingleObservation(Transform3d robotToCamera, Transform3d cameraToTarget, int tagID) {
        return fieldLayout.getTagPose(tagID).map(tagPose -> tagPose
                .transformBy(cameraToTarget.inverse())
                .transformBy(robotToCamera.inverse())
        );
    }

    private Optional<Pose3d> calculateRobotPose3dFromMultiSolvePNPResult(Transform3d robotToCamera, Optional<Transform3d> bestFieldToCamera) {
        return bestFieldToCamera.map(
                fieldToCamera -> new Pose3d()
                        .transformBy(fieldToCamera)
                        .transformBy(robotToCamera.inverse())
        );
    }

    private void calculateVisibleTagsPosesForLog(AprilTagVisionIO.CameraInputs cameraInput, PhotonCameraProperties cameraProperty, Pose2d currentOdometryPose) {
        if (!LOG_DETAILED_FILTERING_DATA) return;
        for (int i = 0; i < cameraInput.fiducialMarksID.length; i++) {
            if (cameraInput.fiducialMarksID[i] == -1) continue;

            fieldLayout.getTagPose(cameraInput.fiducialMarksID[i])
                    .ifPresent(observedVisionTargetPoseInFieldLayout::add);
            observedAprilTagsPoses.add(calculateObservedAprilTagTargetPose(
                    cameraInput.bestCameraToTargets[i],
                    cameraProperty.robotToCamera,
                    currentOdometryPose
            ));
        }
    }

    private final List<Pose3d>
            validRobotPoseEstimationsMultiTag = new ArrayList<>(),
            validRobotPoseEstimationsSingleTag = new ArrayList<>(),
            invalidRobotPoseEstimations = new ArrayList<>();
    private void applyFilteringToRawRobotPose3dEstimations() {
        validRobotPoseEstimationsMultiTag.clear(); validRobotPoseEstimationsSingleTag.clear(); invalidRobotPoseEstimations.clear();
        for (final Pose3d estimation : robotPose3dObservationsMultiTag)
            if (filter.isResultValid(estimation)) validRobotPoseEstimationsMultiTag.add(estimation);
            else invalidRobotPoseEstimations.add(estimation);

        for (final Pose3d estimation : robotPose3dObservationsSingleTag)
            if (filter.isResultValid(estimation)) validRobotPoseEstimationsSingleTag.add(estimation);
            else invalidRobotPoseEstimations.add(estimation);
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

        if (LOG_DETAILED_FILTERING_DATA)
            logFilteringData();

        return getEstimationResultFromValidObservations();
    }

    private Optional<RobotPoseEstimationResult> getEstimationResultFromValidObservations() {
        final boolean resultsCountSufficient =
                validRobotPoseEstimationsSingleTag.size() >= 2
                || (!validRobotPoseEstimationsMultiTag.isEmpty());

        if (!resultsCountSufficient)
            return Optional.empty();

        final List<Statistics.Estimation> robotPoseEstimationsXMeters = new ArrayList<>(),
                robotPoseEstimationsYMeters = new ArrayList<>(),
                robotPoseEstimationsThetaRadians = new ArrayList<>();

        for (Pose3d robotPoseEstimationSingleTag:validRobotPoseEstimationsSingleTag) {
            robotPoseEstimationsXMeters.add(new Statistics.Estimation(
                    robotPoseEstimationSingleTag.getX(),
                    TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION
            ));
            robotPoseEstimationsYMeters.add(new Statistics.Estimation(
                    robotPoseEstimationSingleTag.getY(),
                    TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION
            ));
            robotPoseEstimationsThetaRadians.add(new Statistics.Estimation(
                    robotPoseEstimationSingleTag.getRotation().getZ(),
                    ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION
            ));
        }

        for (Pose3d robotPoseEstimationMultiTag:validRobotPoseEstimationsMultiTag) {
            robotPoseEstimationsXMeters.add(new Statistics.Estimation(
                    robotPoseEstimationMultiTag.getX(),
                    TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_MULTITAG
            ));
            robotPoseEstimationsYMeters.add(new Statistics.Estimation(
                    robotPoseEstimationMultiTag.getY(),
                    TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_MULTITAG
            ));
            robotPoseEstimationsThetaRadians.add(new Statistics.Estimation(
                    robotPoseEstimationMultiTag.getRotation().getZ(),
                    ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_MULTITAG
            ));
        }

        final Statistics.Estimation robotPoseFinalEstimationXMeters = Statistics.linearFilter(robotPoseEstimationsXMeters),
                robotPoseFinalEstimationYMeters = Statistics.linearFilter(robotPoseEstimationsYMeters),
                robotPoseFinalEstimationThetaRadians = Statistics.linearFilter(robotPoseEstimationsThetaRadians);

        final Translation2d translationPointEstimate = new Translation2d(
                robotPoseFinalEstimationXMeters.center(),
                robotPoseFinalEstimationYMeters.center()
        );
        final Rotation2d rotationPointEstimate = Rotation2d.fromRadians(
                robotPoseFinalEstimationThetaRadians.center()
        );

        final double estimationStandardErrorX = robotPoseFinalEstimationXMeters.standardDeviation(),
                estimationStandardErrorY = robotPoseFinalEstimationYMeters.standardDeviation(),
                estimationStandardErrorTheta = robotPoseFinalEstimationThetaRadians.standardDeviation();

        Logger.recordOutput("Vision/MeasurementErrors/translationalStandardError", Math.hypot(estimationStandardErrorX, estimationStandardErrorY));
        Logger.recordOutput("Vision/MeasurementErrors/rotationalStandardError", Math.toDegrees(estimationStandardErrorTheta));

        final double translationStdDev = Math.hypot(
                Statistics.getStandardDeviation(robotPoseEstimationsXMeters),
                Statistics.getStandardDeviation(robotPoseEstimationsYMeters)
        );
        final double rotationStdDev = Statistics.getStandardDeviation(robotPoseEstimationsThetaRadians);
        Logger.recordOutput("Vision/MeasurementErrors/translationalStdDev", translationStdDev);
        Logger.recordOutput("Vision/MeasurementErrors/rotationalStdDev", Math.toDegrees(rotationStdDev));
        if (translationStdDev > TRANSLATIONAL_STANDARD_DEVS_THRESHOLD_DISCARD_RESULT || rotationStdDev > ROTATIONAL_STANDARD_DEVS_THRESHOLD_DISCARD_RESULT)
            return Optional.empty();

        return Optional.of(new RobotPoseEstimationResult(
                new Pose2d(translationPointEstimate, rotationPointEstimate),
                estimationStandardErrorX,
                estimationStandardErrorY,
                estimationStandardErrorTheta
        ));
    }

    /**
     * Log the filtering data
     * */
    private void logFilteringData() {
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/CurrentFilterImplementation", filter.getFilterImplementationName());
        if (!LOG_DETAILED_FILTERING_DATA) return;

        /* these are the detailed filtering data, logging them on RobotRIO1.0 is a bad idea, if you want them, replay the log */
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/ValidPoseEstimationsSingleTags", validRobotPoseEstimationsSingleTag.toArray(Pose3d[]::new));
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/ValidPoseEstimationsMultiTags", validRobotPoseEstimationsMultiTag.toArray(Pose3d[]::new));
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/InvalidPoseEstimations", invalidRobotPoseEstimations.toArray(Pose3d[]::new));
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/VisibleFieldTargets", observedVisionTargetPoseInFieldLayout.toArray(Pose3d[]::new));
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Filtering/AprilTagsObservedPositions/", observedAprilTagsPoses.toArray(Pose3d[]::new));
    }
}

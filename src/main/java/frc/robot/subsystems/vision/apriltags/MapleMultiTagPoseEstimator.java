package frc.robot.subsystems.vision.apriltags;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.LogPaths.APRIL_TAGS_VISION_PATH;
import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.utils.CustomMaths.Statistics;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class MapleMultiTagPoseEstimator {
    private OptionalInt tagToFocus;
    private List<Integer> cameraToFocus;
    private final AprilTagFieldLayout fieldLayout;
    private final VisionResultsFilter filter;
    private final List<PhotonCameraProperties> camerasProperties;

    public MapleMultiTagPoseEstimator(
            AprilTagFieldLayout aprilTagFieldLayout,
            VisionResultsFilter filter,
            List<PhotonCameraProperties> camerasProperties) {
        this.fieldLayout = aprilTagFieldLayout;
        this.filter = filter;
        this.camerasProperties = camerasProperties;
        tagToFocus = OptionalInt.empty();
        cameraToFocus = List.of();
    }

    public void setFocusMode(OptionalInt tagToFocus, Integer... cameraToFocus) {
        this.tagToFocus = tagToFocus;
        this.cameraToFocus = Arrays.asList(cameraToFocus);
    }

    public void enableFocusMode(int tagIdToFocus, Integer... cameraToFocus) {
        setFocusMode(OptionalInt.of(tagIdToFocus), cameraToFocus);
    }

    public void disableFocusMode() {
        this.tagToFocus = OptionalInt.empty();
        this.cameraToFocus = List.of();
    }

    final List<Pose3d> robotPose3dObservationsMultiTag = new ArrayList<>(),
            robotPose3dObservationsSingleTag = new ArrayList<>(),
            observedAprilTagsPoses = new ArrayList<>(),
            observedVisionTargetPoseInFieldLayout = new ArrayList<>();

    private void fetchRobotPose3dEstimationsFromCameraInputs(AprilTagVisionIO.CameraInputs[] cameraInputs) {
        robotPose3dObservationsMultiTag.clear();
        robotPose3dObservationsSingleTag.clear();
        observedAprilTagsPoses.clear();
        observedVisionTargetPoseInFieldLayout.clear();

        if (cameraInputs.length != camerasProperties.size())
            throw new CameraInputsLengthNotMatchException(cameraInputs.length, camerasProperties.size());

        for (int i = 0; i < cameraInputs.length; i++)
            if (cameraInputs[i].newPipeLineResultAvailable)
                fetchSingleCameraInputs(cameraInputs[i], camerasProperties.get(i));
    }

    private void fetchSingleCameraInputs(
            AprilTagVisionIO.CameraInputs cameraInput, PhotonCameraProperties cameraProperty) {

        calculateVisibleTagsPosesForLog(cameraInput, cameraProperty);

        /* add multi-solvepnp result if present */
        if (cameraInput.fieldToCameraResultPresent)
            robotPose3dObservationsMultiTag.add(calculateRobotPose3dFromMultiSolvePNPResult(
                    cameraProperty.robotToCamera, cameraInput.bestFieldToCamera));

        calculateRobotPose3dFromSingleObservation(
                        cameraInput.cameraID,
                        cameraInput.bestTargetTagID,
                        cameraProperty.robotToCamera,
                        cameraInput.bestTargetCameraToTarget,
                        cameraInput.bestTargetAmbiguity)
                .ifPresent(robotPose3dObservationsSingleTag::add);
    }

    private Pose3d calculateObservedAprilTagTargetPose(
            Transform3d bestCameraToTarget, Transform3d robotToCamera, Pose2d currentOdometryPose) {
        return new Pose3d(currentOdometryPose).transformBy(robotToCamera).transformBy(bestCameraToTarget);
    }

    private Optional<Pose3d> calculateRobotPose3dFromSingleObservation(
            int cameraID, int tagID, Transform3d robotToCamera, Transform3d cameraToTarget, double tagAmbiguity) {
        // Ignore result if too far
        if (shouldDiscardTagObservation(cameraID, tagID, robotToCamera, cameraToTarget, tagAmbiguity))
            return Optional.empty();

        return fieldLayout.getTagPose(tagID).map(tagPose -> tagPose.transformBy(cameraToTarget.inverse())
                .transformBy(robotToCamera.inverse()));
    }

    private Pose3d calculateRobotPose3dFromMultiSolvePNPResult(
            Transform3d robotToCamera, Transform3d bestFieldToCamera) {
        return new Pose3d().transformBy(bestFieldToCamera).transformBy(robotToCamera.inverse());
    }

    private boolean shouldDiscardTagObservation(
            int cameraID, int tagID, Transform3d robotToCamera, Transform3d cameraToTarget, double tagAmbiguity) {
        boolean invalidTag = tagID == -1;
        boolean notTheRightTag = tagToFocus.isPresent() && tagToFocus.getAsInt() != tagID;
        boolean rightCamera = cameraToFocus.isEmpty() || cameraToFocus.contains(cameraID);
        if (invalidTag || notTheRightTag || !rightCamera) return true;

        boolean tooFar = cameraToTarget.getTranslation().getNorm() > MAX_TAG_DISTANCE.in(Meters);
        boolean tooMuchAmbiguity = tagAmbiguity > MAX_TAG_AMBIGUITY;

        Transform3d correctlyOrientedCameraToTarget3d = cameraToTarget.plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(
                        robotToCamera.getRotation().getX(),
                        robotToCamera.getRotation().getY(),
                        0)));
        Transform2d cameraToTagBack = new Transform2d(
                correctlyOrientedCameraToTarget3d.getTranslation().toTranslation2d(),
                correctlyOrientedCameraToTarget3d.getRotation().toRotation2d().plus(Rotation2d.k180deg));
        Rotation2d cameraToTargetTranslationDirection =
                cameraToTagBack.getTranslation().getAngle();
        Rotation2d cameraToTagBackRotation = cameraToTagBack.getRotation();
        Angle tagAngle = cameraToTagBackRotation
                .minus(cameraToTargetTranslationDirection)
                .getMeasure();
        boolean angleTooBig = tagAngle.abs(Radians) > MAX_TAG_ANGLE.in(Radians);
        String logPath = APRIL_TAGS_VISION_PATH + "Filtering/TagObservations/Camera-" + cameraID + "/Tag-" + tagID;
        Logger.recordOutput(logPath + "/Ambiguity", tagAmbiguity);
        Logger.recordOutput(logPath + "/TagAngle (Deg)", tagAngle.abs(Degrees));

        return tooFar || tooMuchAmbiguity || angleTooBig;
    }

    private void calculateVisibleTagsPosesForLog(
            AprilTagVisionIO.CameraInputs cameraInput, PhotonCameraProperties cameraProperty) {
        if (!Robot.LOG_DETAILS) return;
        if (shouldDiscardTagObservation(
                cameraInput.cameraID,
                cameraInput.bestTargetTagID,
                cameraProperty.robotToCamera,
                cameraInput.bestTargetCameraToTarget,
                cameraInput.bestTargetAmbiguity)) return;

        fieldLayout.getTagPose(cameraInput.bestTargetTagID).ifPresent(observedVisionTargetPoseInFieldLayout::add);
        observedAprilTagsPoses.add(calculateObservedAprilTagTargetPose(
                cameraInput.bestTargetCameraToTarget,
                cameraProperty.robotToCamera,
                RobotState.getInstance().getVisionPose()));
    }

    private final List<Pose3d> validRobotPoseEstimationsMultiTag = new ArrayList<>(),
            validRobotPoseEstimationsSingleTag = new ArrayList<>(),
            invalidRobotPoseEstimations = new ArrayList<>();

    private void applyFilteringToRawRobotPose3dEstimations() {
        validRobotPoseEstimationsMultiTag.clear();
        validRobotPoseEstimationsSingleTag.clear();
        invalidRobotPoseEstimations.clear();
        for (final Pose3d estimation : robotPose3dObservationsMultiTag)
            if (filter.isResultValid(estimation)) validRobotPoseEstimationsMultiTag.add(estimation);
            else invalidRobotPoseEstimations.add(estimation);

        for (final Pose3d estimation : robotPose3dObservationsSingleTag)
            if (filter.isResultValid(estimation)) validRobotPoseEstimationsSingleTag.add(estimation);
            else invalidRobotPoseEstimations.add(estimation);
    }

    /**
     * using the filtering mechanism, find out the best guess of the robot pose and the standard error
     *
     * @param cameraInputs the inputs of the cameras
     * @return (optionally) the best guess of the robot pose and the standard error, if there are valid targets
     */
    public Optional<VisionObservation> estimateRobotPose(
            AprilTagVisionIO.CameraInputs[] cameraInputs, double timeStampSeconds) {
        if (cameraInputs.length != camerasProperties.size())
            throw new IllegalStateException("camera inputs length"
                    + cameraInputs.length
                    + " does not match cameras properties length: "
                    + camerasProperties.size());

        fetchRobotPose3dEstimationsFromCameraInputs(cameraInputs);

        applyFilteringToRawRobotPose3dEstimations();

        if (Robot.LOG_DETAILS) logFilteringData();

        return getEstimationResultFromValidObservations(timeStampSeconds);
    }

    private Optional<VisionObservation> getEstimationResultFromValidObservations(double timeStampSeconds) {
        //        boolean singleTagEstimationsMoreThan1 = validRobotPoseEstimationsSingleTag.size() >= 2;
        //        boolean multiTagEstimationPresent = !validRobotPoseEstimationsMultiTag.isEmpty();
        //        boolean focusModeEnabledAndSingleTagResultPresent =
        //                tagToFocus.isPresent() && (!validRobotPoseEstimationsSingleTag.isEmpty());
        //        boolean resultsCountSufficient =
        //                singleTagEstimationsMoreThan1 || multiTagEstimationPresent ||
        // focusModeEnabledAndSingleTagResultPresent;
        boolean resultsCountSufficient =
                validRobotPoseEstimationsSingleTag.size() + validRobotPoseEstimationsMultiTag.size() > 0;

        if (!resultsCountSufficient) return Optional.empty();

        final List<Statistics.Estimation> robotPoseEstimationsXMeters = new ArrayList<>(),
                robotPoseEstimationsYMeters = new ArrayList<>();
        final List<Statistics.RotationEstimation> robotPoseEstimationsTheta = new ArrayList<>();

        for (Pose3d robotPoseEstimationSingleTag : validRobotPoseEstimationsSingleTag) {
            Distance translationalStandardError = tagToFocus.isPresent()
                    ? TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_FOCUSED_TAG
                    : TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION;
            Angle rotationalStandardError = tagToFocus.isPresent()
                    ? ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_FOCUSED_TAG
                    : ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION;
            robotPoseEstimationsXMeters.add(new Statistics.Estimation(
                    robotPoseEstimationSingleTag.getX(), translationalStandardError.in(Meters)));
            robotPoseEstimationsYMeters.add(new Statistics.Estimation(
                    robotPoseEstimationSingleTag.getY(), translationalStandardError.in(Meters)));
            robotPoseEstimationsTheta.add(new Statistics.RotationEstimation(
                    robotPoseEstimationSingleTag.getRotation().toRotation2d(), rotationalStandardError.in(Radians)));
        }

        for (Pose3d robotPoseEstimationMultiTag : validRobotPoseEstimationsMultiTag) {
            robotPoseEstimationsXMeters.add(new Statistics.Estimation(
                    robotPoseEstimationMultiTag.getX(), TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_MULTITAG.in(Meters)));
            robotPoseEstimationsYMeters.add(new Statistics.Estimation(
                    robotPoseEstimationMultiTag.getY(), TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_MULTITAG.in(Meters)));
            robotPoseEstimationsTheta.add(new Statistics.RotationEstimation(
                    robotPoseEstimationMultiTag.getRotation().toRotation2d(),
                    ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_MULTITAG.in(Radians)));
        }

        final Statistics.Estimation
                robotPoseFinalEstimationXMeters = Statistics.linearFilter(robotPoseEstimationsXMeters),
                robotPoseFinalEstimationYMeters = Statistics.linearFilter(robotPoseEstimationsYMeters);
        final Statistics.RotationEstimation robotPoseFinalEstimationTheta =
                Statistics.rotationFilter(robotPoseEstimationsTheta);

        final Translation2d translationPointEstimate =
                new Translation2d(robotPoseFinalEstimationXMeters.center(), robotPoseFinalEstimationYMeters.center());
        final Rotation2d rotationPointEstimate = robotPoseFinalEstimationTheta.center();

        final double estimationStandardDevsX = robotPoseFinalEstimationXMeters.standardDeviation(),
                estimationStandardDevsY = robotPoseFinalEstimationYMeters.standardDeviation(),
                estimationStandardErrorTheta = robotPoseFinalEstimationTheta.standardDeviationRad();

        Logger.recordOutput(
                "Vision/MeasurementErrors/translationalStandardDevs",
                Math.hypot(estimationStandardDevsX, estimationStandardDevsY));
        Logger.recordOutput(
                "Vision/MeasurementErrors/rotationalStandardDevs", Math.toDegrees(estimationStandardErrorTheta));

        return Optional.of(VisionObservation.create(
                new Pose2d(translationPointEstimate, rotationPointEstimate),
                Math.hypot(estimationStandardDevsX, estimationStandardDevsY),
                estimationStandardErrorTheta,
                timeStampSeconds));
    }

    /** Log the filtering data */
    private void logFilteringData() {
        Logger.recordOutput(
                APRIL_TAGS_VISION_PATH + "Filtering/CurrentFilterImplementation", filter.getFilterImplementationName());
        if (!Robot.LOG_DETAILS) return;

        /* these are the detailed filtering data, logging them on RobotRIO1.0 is a bad idea, if you want them, replay the log */
        Logger.recordOutput(
                APRIL_TAGS_VISION_PATH + "Filtering/ValidPoseEstimationsSingleTags",
                validRobotPoseEstimationsSingleTag.toArray(Pose3d[]::new));
        Logger.recordOutput(
                APRIL_TAGS_VISION_PATH + "Filtering/ValidPoseEstimationsMultiTags",
                validRobotPoseEstimationsMultiTag.toArray(Pose3d[]::new));
        Logger.recordOutput(
                APRIL_TAGS_VISION_PATH + "Filtering/InvalidPoseEstimations",
                invalidRobotPoseEstimations.toArray(Pose3d[]::new));
        Logger.recordOutput(
                APRIL_TAGS_VISION_PATH + "Filtering/VisibleFieldTargets",
                observedVisionTargetPoseInFieldLayout.toArray(Pose3d[]::new));
        Logger.recordOutput(
                APRIL_TAGS_VISION_PATH + "Filtering/AprilTagsObservedPositions/",
                observedAprilTagsPoses.toArray(Pose3d[]::new));
    }

    public int validPoseEstimationsCount() {
        return validRobotPoseEstimationsMultiTag.size() + validRobotPoseEstimationsSingleTag.size();
    }

    public record VisionObservation(Pose2d visionPose, Matrix<N3, N1> stdDevs, double timestamp) {
        public static VisionObservation create(
                Pose2d visionPose,
                double translationalStandardDeviation,
                double rotationalStandardDeviation,
                double timestamp) {
            return new VisionObservation(
                    visionPose,
                    VecBuilder.fill(
                            translationalStandardDeviation,
                            translationalStandardDeviation,
                            rotationalStandardDeviation),
                    timestamp);
        }
    }

    public static final class CameraInputsLengthNotMatchException extends IllegalStateException {
        public CameraInputsLengthNotMatchException(int cameraInputsLength, int cameraPropertiesLength) {
            super("camera inputs length"
                    + cameraInputsLength
                    + " does not match cameras properties length: "
                    + cameraPropertiesLength);
        }
    }
}

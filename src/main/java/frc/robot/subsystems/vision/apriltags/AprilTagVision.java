package frc.robot.subsystems.vision.apriltags;

import frc.robot.subsystems.MapleSubsystem;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.Config.PhotonCameraProperties;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import static frc.robot.Constants.VisionConfigs.*;
import static frc.robot.Constants.LogConfigs.APRIL_TAGS_VISION_PATH;
import static frc.robot.subsystems.vision.apriltags.MapleMultiTagPoseEstimator.RobotPoseEstimationResult;

public class AprilTagVision extends MapleSubsystem {
    private final AprilTagVisionIO io;
    private final AprilTagVisionIO.VisionInputs inputs;

    private final MapleMultiTagPoseEstimator multiTagPoseEstimator;
    private final HolonomicDriveSubsystem driveSubsystem;
    public AprilTagVision(AprilTagVisionIO io, List<PhotonCameraProperties> camerasProperties, HolonomicDriveSubsystem driveSubsystem) {
        super("Vision");
        this.io = io;
        this.inputs = new AprilTagVisionIO.VisionInputs(camerasProperties.size());

        this.multiTagPoseEstimator = new MapleMultiTagPoseEstimator(
                fieldLayout,
                new CameraHeightFilter(),
                camerasProperties
        );
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void onReset() {

    }

    @Override
    public void periodic(double dt, boolean enabled) {
        io.updateInputs(inputs);
        Logger.processInputs(APRIL_TAGS_VISION_PATH + "Inputs", inputs);

        Optional<RobotPoseEstimationResult> result = multiTagPoseEstimator.estimateRobotPose(inputs.camerasInputs, driveSubsystem.getPose());
        result.ifPresent(robotPoseEstimationResult -> driveSubsystem.addVisionMeasurement(
                robotPoseEstimationResult.pointEstimation,
                getResultsTimeStamp(),
                robotPoseEstimationResult.estimationStandardError
        ));

        Logger.recordOutput(
                APRIL_TAGS_VISION_PATH + "Results/Estimated Pose",
                result.map(robotPoseEstimationResult -> robotPoseEstimationResult.pointEstimation).orElse(null)
        );
        Logger.recordOutput(
                APRIL_TAGS_VISION_PATH + "Results/Standard Error",
                result.map(printStandardError).orElse(null)
        );
    }

    private double getResultsTimeStamp() {
        return inputs.inputsFetchedRealTimeStampSeconds - getResultsAverageLatencySeconds(inputs.camerasInputs);
    }
    private static double getResultsAverageLatencySeconds(AprilTagVisionIO.CameraInputs[] camerasInputs) {
        if (camerasInputs.length == 0)
            return 0;
        double totalLatencyMS = 0;
        for (AprilTagVisionIO.CameraInputs cameraInputs:camerasInputs)
            totalLatencyMS += cameraInputs.resultsDelaySeconds;

        return totalLatencyMS / camerasInputs.length;
    }

    private static final Function<RobotPoseEstimationResult, String> printStandardError = result ->
        String.format(
                "Standard Error : { \n"
                + "Translation X (Meters) %.2f, \n"
                + "Translation Y (Meters) %.2f, \n"
                + "Rotation Theta (Degrees) %.2f, \n }",
                result.estimationStandardError.get(0, 0),
                result.estimationStandardError.get(1, 0),
                Math.toDegrees(result.estimationStandardError.get(2, 0))
        );
}

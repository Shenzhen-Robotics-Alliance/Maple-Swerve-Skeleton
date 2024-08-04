package frc.robot.subsystems.vision.apriltags;

import frc.robot.subsystems.MapleSubsystem;
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
    private final ApriltagVisionIO io;
    private final ApriltagVisionIO.VisionInputs inputs;

    private final MapleMultiTagPoseEstimator multiTagPoseEstimator;
    private final VisionLocalizationResultConsumer consumer;
    public AprilTagVision(ApriltagVisionIO io, List<PhotonCameraProperties> camerasProperties, VisionLocalizationResultConsumer consumer) {
        super("Vision");
        this.io = io;
        this.inputs = new ApriltagVisionIO.VisionInputs(camerasProperties.size());

        this.multiTagPoseEstimator = new MapleMultiTagPoseEstimator(
                fieldLayout,
                new CameraHeightFilter(),
                camerasProperties
        );
        this.consumer = consumer;
    }

    @Override
    public void onReset() {

    }

    @Override
    public void periodic(double dt, boolean enabled) {
        io.updateInputs(inputs);
        Logger.processInputs(APRIL_TAGS_VISION_PATH + "Inputs", inputs);

        Optional<RobotPoseEstimationResult> result = multiTagPoseEstimator.estimateRobotPose(inputs.pipelineResults);
        result.ifPresent(robotPoseEstimationResult -> consumer.addVisionMeasurement(
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
        return inputs.inputsFetchedRealTimeStampSeconds - getResultsAverageLatencySeconds(inputs.pipelineResults);
    }
    private static double getResultsAverageLatencySeconds(PhotonPipelineResult[] results) {
        if (results.length == 0)
            return 0;
        double totalLatencyMS = 0;
        for (PhotonPipelineResult result:results)
            totalLatencyMS += result.getLatencyMillis();

        return totalLatencyMS / results.length / 1000.0;
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

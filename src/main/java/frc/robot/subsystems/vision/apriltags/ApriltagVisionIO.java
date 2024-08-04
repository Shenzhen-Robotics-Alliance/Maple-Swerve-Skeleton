package frc.robot.subsystems.vision.apriltags;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

public interface ApriltagVisionIO {
    int MAX_SUPPORTED_CAMERA_AMOUNT = 10;
    class VisionInputs implements LoggableInputs {
        public final int camerasAmount;
        public final PhotonPipelineResult[] pipelineResults;
        public final boolean[] camerasConnected;
        public double inputsFetchedRealTimeStampSeconds = 0;

        public VisionInputs(int camerasAmount) {
            this.camerasAmount = camerasAmount;
            this.pipelineResults = new PhotonPipelineResult[camerasAmount];
            this.camerasConnected = new boolean[camerasAmount];
        }

        @Override
        public void toLog(LogTable table) {
            table.put("camerasAmount", camerasAmount);
            table.put("inputsFetchedTimeStamp", inputsFetchedRealTimeStampSeconds);
            for (int i = 0; i < camerasAmount; i++) {
                table.put("camera" + i + "PipelineResult", pipelineResults[i]);
                table.put("camera" + i + "Connected", camerasConnected[i]);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            final int loggedCamerasAmount = table.get("camerasAmount", 0);
            if (table.get("camerasAmount", 0) != camerasAmount)
                throw new IllegalStateException("cameras amount from log (" + loggedCamerasAmount +") does not match the settings in replay"
                + "\n check if the code have changed");

            inputsFetchedRealTimeStampSeconds = table.get("inputsFetchedTimeStamp", 0.0);
            for (int i = 0; i < camerasAmount; i++) {
                this.pipelineResults[i] = table.get("camera" + i + "PipelineResult", new PhotonPipelineResult());
                this.camerasConnected[i] = table.get("camera" + i + "Connected", false);
            }
        }
    }

    void updateInputs(VisionInputs inputs);

    default void close() {}

    default void open() {}
}

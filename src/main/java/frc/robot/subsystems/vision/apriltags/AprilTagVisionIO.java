package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Arrays;

public interface AprilTagVisionIO {
    int MAX_SUPPORTED_CAMERA_AMOUNT = 10;
    class CameraInputs {
        public final boolean cameraConnected;
        public final double resultsDelaySeconds;
        public final int currentTargetsCount;
        public final int[] fiducialMarksID;
        public final Transform3d[] bestCameraToTargets;

        public CameraInputs(boolean cameraConnected, double resultsDelaySeconds, int currentTargetsCount, int[] fiducialMarksID, Transform3d[] bestCameraToTargets) {
            this.cameraConnected = cameraConnected;
            this.resultsDelaySeconds = resultsDelaySeconds;
            this.currentTargetsCount = currentTargetsCount;
            this.fiducialMarksID = fiducialMarksID;
            this.bestCameraToTargets = bestCameraToTargets;
        }

        public static CameraInputs empty = new CameraInputs(false, 0, 0, new int[0], new Transform3d[0]);
        public static CameraInputs fromPhotonPipeLine(PhotonPipelineResult pipelineResult, boolean cameraConnected) {
            return new CameraInputs(
                    cameraConnected,
                    pipelineResult.getLatencyMillis() / 1000.0,
                    pipelineResult.getTargets().size(),
                    pipelineResult.getTargets().stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray(),
                    pipelineResult.getTargets().stream().map(PhotonTrackedTarget::getBestCameraToTarget).toArray(Transform3d[]::new)
            );
        }

        public static CameraInputs fromLog(LogTable table, int cameraID) {
            final String cameraKey = "camera" + cameraID;
            return new CameraInputs(
                    table.get(cameraKey+"Connected", false),
                    table.get(cameraKey+"ResultsDelaySeconds", 0.0),
                    table.get(cameraKey+"CurrentTargetsCount", 0),
                    table.get(cameraKey+"FiducialMarksID", new int[0]),
                    table.get(cameraKey+"bestCameraToTargets", new Transform3d[0])
            );
        }

        public void writeToLog(LogTable table, int cameraID) {
            final String cameraKey = "camera" + cameraID;
            table.put(cameraKey+"Connected", cameraConnected);
            table.put(cameraKey+"ResultsDelaySeconds", resultsDelaySeconds);
            table.put(cameraKey+"CurrentTargetsCount", currentTargetsCount);
            table.put(cameraKey+"FiducialMarksID", fiducialMarksID);
            table.put(cameraKey+"bestCameraToTargets", bestCameraToTargets);
        }
    }
    class VisionInputs implements LoggableInputs {
        public final int camerasAmount;
        public final CameraInputs[] camerasInputs;
        public double inputsFetchedRealTimeStampSeconds = 0;

        public VisionInputs(int camerasAmount) {
            this.camerasAmount = camerasAmount;
            this.camerasInputs = new CameraInputs[camerasAmount];
            Arrays.fill(camerasInputs, CameraInputs.empty);
        }

        @Override
        public void toLog(LogTable table) {
            table.put("camerasAmount", camerasAmount);
            table.put("inputsFetchedTimeStamp", inputsFetchedRealTimeStampSeconds);
            for (int i = 0; i < camerasAmount; i++)
                camerasInputs[i].writeToLog(table, i);
        }

        @Override
        public void fromLog(LogTable table) {
            final int loggedCamerasAmount = table.get("camerasAmount", 0);
            if (table.get("camerasAmount", 0) != camerasAmount)
                throw new IllegalStateException("cameras amount from log (" + loggedCamerasAmount +") does not match the settings in replay"
                + "\n check if the code have changed");

            inputsFetchedRealTimeStampSeconds = table.get("inputsFetchedTimeStamp", 0.0);
            for (int i = 0; i < camerasAmount; i++)
                camerasInputs[i] = CameraInputs.fromLog(table, i);
        }
    }

    void updateInputs(VisionInputs inputs);

    default void close() {}

    default void open() {}
}

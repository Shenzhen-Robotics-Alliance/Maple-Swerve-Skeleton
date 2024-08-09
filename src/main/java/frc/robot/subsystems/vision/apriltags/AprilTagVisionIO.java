package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Arrays;

public interface AprilTagVisionIO {
    class CameraInputs {
        public static final int MAX_TARGET_PER_CAMERA = 5;
        public boolean cameraConnected;
        public double resultsDelaySeconds;
        public int currentTargetsCount;
        public final int[] fiducialMarksID;
        public final Transform3d[] bestCameraToTargets;

        public CameraInputs() {
            this.fiducialMarksID = new int[MAX_TARGET_PER_CAMERA];
            this.bestCameraToTargets = new Transform3d[MAX_TARGET_PER_CAMERA];
            clear();
        }

        public void clear() {
            this.cameraConnected = false;
            this.resultsDelaySeconds = 0;
            this.currentTargetsCount = 0;
            Arrays.fill(fiducialMarksID, -1);
            Arrays.fill(bestCameraToTargets, new Transform3d());
        }

        public void fromPhotonPipeLine(PhotonPipelineResult pipelineResult, boolean cameraConnected) {
            this.cameraConnected = cameraConnected;
            this.resultsDelaySeconds = pipelineResult.getLatencyMillis() / 1000.0;
            this.currentTargetsCount = pipelineResult.getTargets().size();
            Arrays.fill(fiducialMarksID, -1);
            Arrays.fill(bestCameraToTargets, new Transform3d());
            for (int i = 0; i < currentTargetsCount && i < MAX_TARGET_PER_CAMERA; i++) {
                this.fiducialMarksID[i] = pipelineResult.getTargets().get(i).getFiducialId();
                this.bestCameraToTargets[i] = pipelineResult.getTargets().get(i).getBestCameraToTarget();
            }
        }

        public void fromLog(LogTable table, int cameraID) {
            final String cameraKey = "camera" + cameraID;
            this.cameraConnected = table.get(cameraKey+"Connected", false);
            this.resultsDelaySeconds = table.get(cameraKey+"ResultsDelaySeconds", 0.0);
            this.currentTargetsCount = table.get(cameraKey+"CurrentTargetsCount", 0);
            final int[] fiducialMarkIDLogged = table.get(cameraKey+"FiducialMarksID", new int[MAX_TARGET_PER_CAMERA]);
            final Transform3d[] bestCameraToTargetsLogged = table.get(cameraKey+"bestCameraToTargets", new Transform3d[MAX_TARGET_PER_CAMERA]);
            if (fiducialMarkIDLogged.length != MAX_TARGET_PER_CAMERA || bestCameraToTargetsLogged.length != MAX_TARGET_PER_CAMERA)
                DriverStation.reportError("vision log length not match", false);
            for (int i = 0; i < MAX_TARGET_PER_CAMERA; i++) {
                fiducialMarksID[i] = fiducialMarkIDLogged[i];
                bestCameraToTargets[i] = bestCameraToTargetsLogged[i];
            }
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
        public boolean coprocessorConnected = false;

        public VisionInputs(int camerasAmount) {
            this.camerasAmount = camerasAmount;
            this.camerasInputs = new CameraInputs[camerasAmount];
            for (int i = 0; i < camerasAmount; i++)
                camerasInputs[i] = new CameraInputs();
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
                camerasInputs[i].fromLog(table, i);
        }
    }

    void updateInputs(VisionInputs inputs);

    default void close() {}

    default void open() {}
}

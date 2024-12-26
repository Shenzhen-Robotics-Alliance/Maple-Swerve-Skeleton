package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

public interface AprilTagVisionIO {
    class CameraInputs {
        public static final int MAX_TARGET_PER_CAMERA = 5;
        public boolean cameraConnected;
        public double timeStampSeconds;
        public int currentTargetsCount;
        public final int[] fiducialMarksID;
        public final Transform3d[] bestCameraToTargets;
        public Optional<Transform3d> bestFieldToCamera = Optional.empty();

        public CameraInputs() {
            this.fiducialMarksID = new int[MAX_TARGET_PER_CAMERA];
            this.bestCameraToTargets = new Transform3d[MAX_TARGET_PER_CAMERA];
            clear(false);
        }

        public void clear(boolean cameraConnected) {
            this.cameraConnected = cameraConnected;
            this.timeStampSeconds = 0;
            this.currentTargetsCount = 0;
            Arrays.fill(fiducialMarksID, -1);
            Arrays.fill(bestCameraToTargets, new Transform3d());
        }

        public void fromPhotonPipeLine(PhotonPipelineResult pipelineResult) {
            this.cameraConnected = true;
            this.timeStampSeconds = pipelineResult.getTimestampSeconds();
            this.currentTargetsCount = Math.min(pipelineResult.getTargets().size(), MAX_TARGET_PER_CAMERA);
            Arrays.fill(fiducialMarksID, -1);
            Arrays.fill(bestCameraToTargets, new Transform3d());
            for (int i = 0; i < currentTargetsCount && i < MAX_TARGET_PER_CAMERA; i++) {
                this.fiducialMarksID[i] = pipelineResult.getTargets().get(i).getFiducialId();
                this.bestCameraToTargets[i] = pipelineResult.getTargets().get(i).getBestCameraToTarget();
            }
            this.bestFieldToCamera = pipelineResult.getMultiTagResult().isPresent()
                    ? Optional.of(pipelineResult.getMultiTagResult().get().estimatedPose.best)
                    : Optional.empty();
        }

        private static final Transform3d NULL_TRANSFORM = new Transform3d(-114514, -114514, -114514, new Rotation3d());

        public void fromLog(LogTable table, int cameraID) {
            final String cameraKey = "Camera" + cameraID;
            this.cameraConnected = table.get(cameraKey + "Connected", false);
            this.timeStampSeconds = table.get(cameraKey + "TimeStampSeconds", 0.0);
            this.currentTargetsCount = table.get(cameraKey + "CurrentTargetsCount", 0);
            final int[] fiducialMarkIDLogged = table.get(cameraKey + "FiducialMarksID", new int[MAX_TARGET_PER_CAMERA]);
            final Transform3d[] bestCameraToTargetsLogged =
                    table.get(cameraKey + "BestCameraToTargets", new Transform3d[MAX_TARGET_PER_CAMERA]);
            if (fiducialMarkIDLogged.length != MAX_TARGET_PER_CAMERA
                    || bestCameraToTargetsLogged.length != MAX_TARGET_PER_CAMERA)
                DriverStation.reportError("vision log length not match", false);
            for (int i = 0; i < MAX_TARGET_PER_CAMERA; i++) {
                fiducialMarksID[i] = fiducialMarkIDLogged[i];
                bestCameraToTargets[i] = bestCameraToTargetsLogged[i];
            }

            if (table.get(cameraKey + "BestCameraToFieldPresents", false))
                this.bestFieldToCamera = Optional.of(table.get(cameraKey + "BestCameraToField", new Transform3d()));
            else this.bestFieldToCamera = Optional.empty();
        }

        public void writeToLog(LogTable table, int cameraID) {
            final String cameraKey = "Camera" + cameraID;
            table.put(cameraKey + "Connected", cameraConnected);
            table.put(cameraKey + "TimeStampSeconds", timeStampSeconds);
            table.put(cameraKey + "CurrentTargetsCount", currentTargetsCount);
            table.put(cameraKey + "FiducialMarksID", fiducialMarksID);
            table.put(cameraKey + "BestCameraToTargets", bestCameraToTargets);
            table.put(cameraKey + "BestCameraToFieldPresents", bestFieldToCamera.isPresent());
            table.put(cameraKey + "BestCameraToField", bestFieldToCamera.orElse(new Transform3d()));
        }
    }

    class VisionInputs implements LoggableInputs {
        public final int camerasAmount;
        public final CameraInputs[] camerasInputs;

        public VisionInputs(int camerasAmount) {
            this.camerasAmount = camerasAmount;
            this.camerasInputs = new CameraInputs[camerasAmount];
            for (int i = 0; i < camerasAmount; i++) camerasInputs[i] = new CameraInputs();
        }

        @Override
        public void toLog(LogTable table) {
            table.put("camerasAmount", camerasAmount);
            for (int i = 0; i < camerasAmount; i++) camerasInputs[i].writeToLog(table, i);
        }

        @Override
        public void fromLog(LogTable table) {
            final int loggedCamerasAmount = table.get("camerasAmount", 0);
            if (table.get("camerasAmount", 0) != camerasAmount)
                throw new IllegalStateException("cameras amount from log ("
                        + loggedCamerasAmount
                        + ") does not match the settings in replay"
                        + "\n check if the code have changed");

            for (int i = 0; i < camerasAmount; i++) camerasInputs[i].fromLog(table, i);
        }
    }

    void updateInputs(VisionInputs inputs);

    default void close() {}

    default void open() {}
}

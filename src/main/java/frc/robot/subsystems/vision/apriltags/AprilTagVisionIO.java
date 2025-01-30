package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface AprilTagVisionIO {
    class CameraInputs {
        public static final int MAX_TARGET_PER_CAMERA = 5;
        public final int cameraID;
        public boolean cameraConnected;
        public boolean newPipeLineResultAvailable;
        public double timeStampSeconds;
        public int currentTargetsCount;
        public final int[] fiducialMarksID;
        public final double[] tagAmbiguities;
        public final Transform3d[] bestCameraToTargets;
        public Optional<Transform3d> bestFieldToCamera = Optional.empty();

        public CameraInputs(int id) {
            this.cameraID = id;
            this.fiducialMarksID = new int[MAX_TARGET_PER_CAMERA];
            this.bestCameraToTargets = new Transform3d[MAX_TARGET_PER_CAMERA];
            this.tagAmbiguities = new double[MAX_TARGET_PER_CAMERA];
            markAsDisconnected();
        }

        public void markAsConnectedButNoResult() {
            this.newPipeLineResultAvailable = false;
            this.cameraConnected = true;
            this.timeStampSeconds = 0;
            this.currentTargetsCount = 0;
            Arrays.fill(fiducialMarksID, -1);
            Arrays.fill(bestCameraToTargets, new Transform3d());
            Arrays.fill(tagAmbiguities, 0.0);
        }

        public void markAsDisconnected() {
            markAsConnectedButNoResult();
            this.cameraConnected = false;
        }

        public void readFromPhotonPipeLine(PhotonPipelineResult pipelineResult) {
            this.newPipeLineResultAvailable = true;
            this.cameraConnected = true;
            this.timeStampSeconds = pipelineResult.getTimestampSeconds();
            this.currentTargetsCount = Math.min(pipelineResult.getTargets().size(), MAX_TARGET_PER_CAMERA);
            Arrays.fill(fiducialMarksID, -1);
            Arrays.fill(bestCameraToTargets, new Transform3d());
            Arrays.fill(tagAmbiguities, 0.0);
            for (int i = 0; i < currentTargetsCount && i < MAX_TARGET_PER_CAMERA; i++) {
                PhotonTrackedTarget target = pipelineResult.getTargets().get(i);
                this.fiducialMarksID[i] = target.getFiducialId();
                this.bestCameraToTargets[i] = target.getBestCameraToTarget();
                this.tagAmbiguities[i] = target.getPoseAmbiguity();
            }
            this.bestFieldToCamera = pipelineResult.getMultiTagResult().isPresent()
                    ? Optional.of(pipelineResult.getMultiTagResult().get().estimatedPose.best)
                    : Optional.empty();
        }

        public void readFromLog(LogTable table) {
            final String cameraKey = "Camera" + cameraID;
            this.cameraConnected = table.get(cameraKey + "Connected", false);
            this.newPipeLineResultAvailable = table.get(cameraKey + "NewPipeLineResultAvailable", false);
            this.timeStampSeconds = table.get(cameraKey + "TimeStampSeconds", 0.0);
            this.currentTargetsCount = table.get(cameraKey + "CurrentTargetsCount", 0);
            final int[] fiducialMarkIDLogged = table.get(cameraKey + "FiducialMarksID", new int[MAX_TARGET_PER_CAMERA]);
            final Transform3d[] bestCameraToTargetsLogged =
                    table.get(cameraKey + "BestCameraToTargets", new Transform3d[MAX_TARGET_PER_CAMERA]);
            final double[] tagAmbiguitiesLogged =
                    table.get(cameraKey + "TagAmbiguities", new double[MAX_TARGET_PER_CAMERA]);
            for (int i = 0; i < MAX_TARGET_PER_CAMERA; i++) {
                fiducialMarksID[i] = fiducialMarkIDLogged[i];
                bestCameraToTargets[i] = bestCameraToTargetsLogged[i];
                tagAmbiguities[i] = tagAmbiguitiesLogged[i];
            }

            if (table.get(cameraKey + "BestCameraToFieldPresents", false))
                this.bestFieldToCamera = Optional.of(table.get(cameraKey + "BestCameraToField", new Transform3d()));
            else this.bestFieldToCamera = Optional.empty();
        }

        public void writeToLog(LogTable table) {
            final String cameraKey = "Camera" + cameraID;
            table.put(cameraKey + "Connected", cameraConnected);
            table.put(cameraKey + "NewPipeLineResultAvailable", this.newPipeLineResultAvailable);
            table.put(cameraKey + "TimeStampSeconds", timeStampSeconds);
            table.put(cameraKey + "CurrentTargetsCount", currentTargetsCount);
            table.put(cameraKey + "FiducialMarksID", fiducialMarksID);
            table.put(cameraKey + "BestCameraToTargets", bestCameraToTargets);
            table.put(cameraKey + "TagAmbiguities", tagAmbiguities);
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
            for (int i = 0; i < camerasAmount; i++) camerasInputs[i] = new CameraInputs(i);
        }

        @Override
        public void toLog(LogTable table) {
            table.put("camerasAmount", camerasAmount);
            Arrays.stream(camerasInputs).forEach(cameraInputs -> cameraInputs.writeToLog(table));
        }

        @Override
        public void fromLog(LogTable table) {
            final int loggedCamerasAmount = table.get("camerasAmount", 0);
            if (table.get("camerasAmount", 0) != camerasAmount)
                throw new IllegalStateException("cameras amount from log ("
                        + loggedCamerasAmount
                        + ") does not match the settings in replay"
                        + "\n check if the code have changed");

            Arrays.stream(camerasInputs).forEach(cameraInputs -> cameraInputs.readFromLog(table));
        }
    }

    void updateInputs(VisionInputs inputs);

    default void close() {}

    default void open() {}
}

package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface AprilTagVisionIO {
    class CameraInputs implements LoggableInputs {
        public final int cameraID;
        public boolean cameraConnected;
        public boolean newPipeLineResultAvailable;
        public double timeStampSeconds;
        public int bestTargetTagID;
        public double bestTargetAmbiguity;
        public Transform3d bestTargetCameraToTarget;
        public Transform3d bestTargetAlternativeCameraToTarget;

        public boolean fieldToCameraResultPresent;
        public Transform3d bestFieldToCamera;

        public CameraInputs(int id) {
            this.cameraID = id;
            markAsDisconnected();
        }

        public void markAsConnectedButNoResult() {
            this.bestTargetTagID = -1;
            this.bestTargetCameraToTarget = new Transform3d();
            this.bestTargetAlternativeCameraToTarget = new Transform3d();
            this.bestTargetAmbiguity = 0.0;
            this.newPipeLineResultAvailable = false;
            this.cameraConnected = true;
            this.timeStampSeconds = 0;
            this.fieldToCameraResultPresent = false;
            this.bestFieldToCamera = new Transform3d();
        }

        public void markAsDisconnected() {
            markAsConnectedButNoResult();
            this.cameraConnected = false;
        }

        public void readFromPhotonPipeLine(PhotonPipelineResult pipelineResult) {
            this.newPipeLineResultAvailable = true;
            this.cameraConnected = true;
            this.timeStampSeconds = pipelineResult.getTimestampSeconds();

            PhotonTrackedTarget bestTarget = pipelineResult.getBestTarget();
            if (bestTarget != null) {
                this.bestTargetTagID = bestTarget.getFiducialId();
                this.bestTargetCameraToTarget = bestTarget.getBestCameraToTarget();
                this.bestTargetAlternativeCameraToTarget = bestTarget.getAlternateCameraToTarget();
                this.bestTargetAmbiguity = bestTarget.getPoseAmbiguity();
            } else {
                this.bestTargetTagID = -1;
                this.bestTargetCameraToTarget = new Transform3d();
                this.bestTargetAlternativeCameraToTarget = new Transform3d();
                this.bestTargetAmbiguity = 0.0;
            }

            this.fieldToCameraResultPresent = pipelineResult.getMultiTagResult().isPresent();
            if (fieldToCameraResultPresent)
                this.bestFieldToCamera = pipelineResult.getMultiTagResult().get().estimatedPose.best;
            else this.bestFieldToCamera = new Transform3d();
        }

        @Override
        public void toLog(LogTable table) {
            table.put("CameraConnected", cameraConnected);
            table.put("NewPipeLineResultAvailable", newPipeLineResultAvailable);
            table.put("TimeStampSeconds", timeStampSeconds);
            table.put("BestTargetTagID", bestTargetTagID);
            table.put("BestTargetAmbiguity", bestTargetAmbiguity);
            table.put("BestTargetCameraToTarget", bestTargetCameraToTarget);
            table.put("BestTargetAlternativeCameraToTarget", bestTargetAlternativeCameraToTarget);
            table.put("FieldToCameraResultPresent", fieldToCameraResultPresent);
            table.put("BestFieldToCamera", bestFieldToCamera);
        }

        @Override
        public void fromLog(LogTable table) {
            cameraConnected = table.get("CameraConnected", cameraConnected);
            newPipeLineResultAvailable = table.get("NewPipeLineResultAvailable", newPipeLineResultAvailable);
            timeStampSeconds = table.get("TimeStampSeconds", timeStampSeconds);
            bestTargetTagID = table.get("BestTargetTagID", bestTargetTagID);
            bestTargetAmbiguity = table.get("BestTargetAmbiguity", bestTargetAmbiguity);
            bestTargetCameraToTarget = table.get("BestTargetCameraToTarget", bestTargetCameraToTarget);
            bestTargetAlternativeCameraToTarget =
                    table.get("BestTargetAlternativeCameraToTarget", bestTargetAlternativeCameraToTarget);
            fieldToCameraResultPresent = table.get("FieldToCameraResultPresent", fieldToCameraResultPresent);
            bestFieldToCamera = table.get("BestFieldToCamera", bestFieldToCamera);
        }
    }

    void updateInputs(CameraInputs... inputs);

    default void close() {}

    default void open() {}
}

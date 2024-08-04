package frc.robot.subsystems.vision.apriltags;

import frc.robot.utils.Config.PhotonCameraProperties;
import frc.robot.utils.MapleTimeUtils;
import org.photonvision.PhotonCamera;

import java.util.List;

public class ApriltagVisionIOReal implements ApriltagVisionIO {
    protected final PhotonCamera[] cameras;
    public ApriltagVisionIOReal(List<PhotonCameraProperties> cameraProperties) {
        if (cameraProperties.size() > MAX_SUPPORTED_CAMERA_AMOUNT)
            throw new IllegalArgumentException("max supported camera count is 10");
        cameras = new PhotonCamera[cameraProperties.size()];

        for (int i = 0; i < cameraProperties.size(); i++)
            cameras[i] = new PhotonCamera(cameraProperties.get(i).name);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        if (inputs.camerasAmount != cameras.length)
            throw new IllegalStateException("inputs camera amount (" + inputs.camerasAmount + ") does not match actual cameras amount");

        for (int i = 0; i < cameras.length; i++) {
            inputs.pipelineResults[i] = cameras[i].getLatestResult();
            inputs.camerasConnected[i] = cameras[i].isConnected();
        }
        inputs.inputsFetchedRealTimeStampSeconds = MapleTimeUtils.getRealTimeSeconds();
    }

    @Override
    public void close() {
        for (PhotonCamera camera:cameras)
            camera.close();
    }
}

package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.net.PortForwarder;
import frc.robot.utils.MapleTimeUtils;
import org.photonvision.PhotonCamera;

import java.util.List;

public class AprilTagVisionIOReal implements AprilTagVisionIO {
    protected final PhotonCamera[] cameras;
    public AprilTagVisionIOReal(List<PhotonCameraProperties> cameraProperties) {
        if (cameraProperties.size() > 16)
            throw new IllegalArgumentException("max supported camera count is 16");
        cameras = new PhotonCamera[cameraProperties.size()];

        for (int i = 0; i < cameraProperties.size(); i++)
            cameras[i] = new PhotonCamera(cameraProperties.get(i).name);

        PortForwarder.add(5800, "photonvision", 5800);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        if (inputs.camerasAmount != cameras.length)
            throw new IllegalStateException("inputs camera amount (" + inputs.camerasAmount + ") does not match actual cameras amount");

        for (int i = 0; i < cameras.length; i++)
            if (cameras[i].isConnected())
                inputs.camerasInputs[i].fromPhotonPipeLine(
                        cameras[i].getLatestResult(),
                        cameras[i].isConnected()
                );
            else inputs.camerasInputs[i].clear();
        inputs.inputsFetchedRealTimeStampSeconds = MapleTimeUtils.getRealTimeSeconds();
    }

    @Override
    public void close() {
        for (PhotonCamera camera:cameras)
            camera.close();
    }
}

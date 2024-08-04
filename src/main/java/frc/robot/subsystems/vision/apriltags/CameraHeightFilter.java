package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.Pose3d;


public class CameraHeightFilter implements VisionResultsFilter  {
    private static final double TOLERANCE_METERS = 0.3;

    @Override
    public String getFilterImplementationName() {
        return "Camera Height Filter, by @caTr1x";
    }

    @Override
    public boolean test(Pose3d pose3d) {
        return Math.abs(pose3d.getZ()) < TOLERANCE_METERS;
    }
}

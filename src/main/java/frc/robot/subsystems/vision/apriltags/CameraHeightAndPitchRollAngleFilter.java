package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.Pose3d;


public class CameraHeightAndPitchRollAngleFilter implements VisionResultsFilter  {
    private static final double ROBOT_HEIGHT_TOLERANCE = 0.3, ROBOT_PITCH_TOLERANCE_RADIANS = Math.toRadians(10), ROBOT_ROLL_TOLERANCE_RADIANS = Math.toRadians(10);

    @Override
    public String getFilterImplementationName() {
        return "Camera Height and Pitch Roll Angle Filter, by @caTr1x";
    }

    @Override
    public boolean test(Pose3d pose3d) {
        return Math.abs(pose3d.getZ()) < ROBOT_HEIGHT_TOLERANCE
                && Math.abs(pose3d.getRotation().getX()) < ROBOT_ROLL_TOLERANCE_RADIANS
                && Math.abs(pose3d.getRotation().getY()) < ROBOT_PITCH_TOLERANCE_RADIANS;
    }
}

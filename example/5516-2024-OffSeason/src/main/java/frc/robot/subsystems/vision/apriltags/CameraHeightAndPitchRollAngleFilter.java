package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.Pose3d;


public class CameraHeightAndPitchRollAngleFilter implements VisionResultsFilter  {
    private static final double ROBOT_HEIGHT_TOLERANCE = 0.12, ROBOT_PITCH_TOLERANCE_RADIANS = Math.toRadians(5), ROBOT_ROLL_TOLERANCE_RADIANS = Math.toRadians(5);

    @Override
    public String getFilterImplementationName() {
        return "Camera Height and Pitch Roll Angle Filter, by @caTr1x";
    }

    @Override
    public boolean isResultValid(Pose3d robotPoseEstimation) {
        return Math.abs(robotPoseEstimation.getZ()) < ROBOT_HEIGHT_TOLERANCE
                && Math.abs(robotPoseEstimation.getRotation().getX()) < ROBOT_ROLL_TOLERANCE_RADIANS
                && Math.abs(robotPoseEstimation.getRotation().getY()) < ROBOT_PITCH_TOLERANCE_RADIANS;
    }
}

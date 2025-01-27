package frc.robot.subsystems.vision.apriltags;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;

public class CameraHeightAndPitchRollAngleFilter implements VisionResultsFilter {

    @Override
    public String getFilterImplementationName() {
        return "Camera Height and Pitch Roll Angle Filter, by @caTr1x";
    }

    @Override
    public boolean isResultValid(Pose3d robotPoseEstimation) {
        return Math.abs(robotPoseEstimation.getZ()) < ROBOT_HEIGHT_TOLERANCE.in(Meters)
                && Math.abs(robotPoseEstimation.getRotation().getX()) < ROBOT_ROLL_TOLERANCE.in(Radians)
                && Math.abs(robotPoseEstimation.getRotation().getY()) < ROBOT_PITCH_TOLERANCE.in(Radians);
    }
}

package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.Pose3d;

import java.util.function.Predicate;

public interface VisionResultsFilter {
    String getFilterImplementationName();

    boolean isResultValid(Pose3d robotPoseEstimation);
}

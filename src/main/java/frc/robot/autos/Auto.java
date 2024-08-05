package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public abstract class Auto extends SequentialCommandGroup {
    public abstract Pose2d getStartingPoseAtBlueAlliance();

    public static Auto none() {
        return new Auto() {@Override public Pose2d getStartingPoseAtBlueAlliance() {
            return new Pose2d(3, 3, new Rotation2d());
        }};
    }
}

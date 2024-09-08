package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.CommandOnFly;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;

import java.util.function.Supplier;

public class PathFindToPose extends CommandOnFly {
    public PathFindToPose(HolonomicDriveSubsystem driveSubsystem, Supplier<Pose2d> targetPose, double speedMultiplier) {
        super(() -> AutoBuilder.pathfindToPose(
                targetPose.get(),
                driveSubsystem.getChassisConstrains(speedMultiplier)
        ));
    }
}

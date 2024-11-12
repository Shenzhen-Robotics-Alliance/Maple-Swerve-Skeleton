package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandOnFly;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.utils.ChassisHeadingController;
import java.util.function.Supplier;

public class PathFindToPose extends CommandOnFly {
    public PathFindToPose(
            HolonomicDriveSubsystem driveSubsystem,
            Supplier<Pose2d> targetPose,
            double speedMultiplier,
            double goalEndVelocity) {
        super(() -> AutoBuilder.pathfindToPose(
                        targetPose.get(), driveSubsystem.getChassisConstrains(speedMultiplier), goalEndVelocity)
                .beforeStarting(Commands.runOnce(() -> SwerveDrive.swerveHeadingController.setHeadingRequest(
                        new ChassisHeadingController.NullRequest()))));
    }
}

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import java.util.function.Supplier;

public class AutoAlignment extends SequentialCommandGroup {
    private static final Pose2d DEFAULT_TOLERANCE = new Pose2d(0.03, 0.03, new Rotation2d(2));

    public AutoAlignment(HolonomicDriveSubsystem driveSubsystem, Supplier<Pose2d> targetPose) {
        this(driveSubsystem, targetPose, targetPose, DEFAULT_TOLERANCE, 0.75, 0.5);
    }

    /**
     * creates a precise auto-alignment command NOTE: AutoBuilder must be configured! the command has two steps: 1.
     * path-find to the target pose, roughly 2. accurate auto alignment
     */
    public AutoAlignment(
            HolonomicDriveSubsystem driveSubsystem,
            Supplier<Pose2d> roughTarget,
            Supplier<Pose2d> target,
            Pose2d tolerance,
            double speedMultiplier,
            double goalEndVelocityRoughApproach) {
        this(
                driveSubsystem,
                roughTarget,
                target,
                tolerance,
                speedMultiplier,
                goalEndVelocityRoughApproach,
                Commands.run(() -> {}),
                Commands.none());
    }

    /**
     * creates a precise auto-alignment command NOTE: AutoBuilder must be configured! the command has two steps: 1.
     * path-find to the target pose, roughly 2. accurate auto alignment
     */
    public AutoAlignment(
            HolonomicDriveSubsystem driveSubsystem,
            Supplier<Pose2d> roughTarget,
            Supplier<Pose2d> target,
            Pose2d tolerance,
            double speedMultiplier,
            double goalEndVelocityRoughApproach,
            Command toRunDuringRoughApproach,
            Command toRunDuringPrecise) {
        final Command
                pathFindToTargetRough =
                        new PathFindToPose(driveSubsystem, roughTarget, speedMultiplier, goalEndVelocityRoughApproach),
                preciseAlignment = new DriveToPose(driveSubsystem, target, tolerance, 2);

        super.addRequirements(driveSubsystem);

        super.addCommands(pathFindToTargetRough.raceWith(toRunDuringRoughApproach));
        super.addCommands(preciseAlignment.alongWith(toRunDuringPrecise));
    }
}

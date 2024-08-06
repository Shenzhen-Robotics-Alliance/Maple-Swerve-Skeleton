package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoAlignment extends SequentialCommandGroup {
    private static final Pose2d DEFAULT_TOLERANCE = new Pose2d(0.03, 0.03, new Rotation2d(2));
    public AutoAlignment(HolonomicDriveSubsystem driveSubsystem, Supplier<Pose2d> targetPose){
        this(driveSubsystem, targetPose, targetPose, DEFAULT_TOLERANCE, 0.75);
    }

    /**
     * creates a precise auto-alignment command
     * NOTE: AutoBuilder must be configured!
     * the command has two steps:
     * 1. path-find to the target pose, roughly
     * 2. accurate auto alignment
     * */
    public AutoAlignment(HolonomicDriveSubsystem driveSubsystem, Supplier<Pose2d> roughTarget, Supplier<Pose2d> target, Pose2d tolerance, double speedMultiplier) {
        final Command pathFindToTargetRough = new PathFindToPose(driveSubsystem, target, speedMultiplier),
                preciseAlignment = new DriveToPosition(
                        driveSubsystem,
                        target,
                        tolerance
                );

        super.addCommands(pathFindToTargetRough);
        super.addCommands(preciseAlignment);

        super.addRequirements(driveSubsystem);
    }
}

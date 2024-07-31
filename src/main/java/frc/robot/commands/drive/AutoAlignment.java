package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoAlignment extends SequentialCommandGroup {
    public AutoAlignment(HolonomicDriveSubsystem driveSubsystem, Pose2d targetPose, Pose2d tolerance) {
        this(
                driveSubsystem.getChassisConstrains(0.75),
                DriveToPosition.createPositionController(),
                driveSubsystem::getPose,
                driveSubsystem::runRobotCentricChassisSpeeds,
                driveSubsystem,
                targetPose,
                tolerance
        );
    }

    /**
     * creates a precise auto-alignment command
     * NOTE: AutoBuilder must be configured!
     * the command has two steps:
     * 1. path-find to the target pose, roughly
     * 2. accurate auto alignment
     * */
    public AutoAlignment(
            PathConstraints constraints,
            HolonomicDriveController holonomicDriveController,
            Supplier<Pose2d> robotPoseSupplier,
            Consumer<ChassisSpeeds> robotRelativeSpeedsOutput,
            Subsystem driveSubsystem,
            Pose2d targetPose,
            Pose2d tolerance
    ) {
        /* tolerance for the precise approach */
        holonomicDriveController.setTolerance(tolerance);
        final Command
                pathFindToTargetRough = AutoBuilder.pathfindToPose(targetPose, constraints, 0.5),
                preciseAlignment = new FunctionalCommand(
                        () -> {},
                        () -> robotRelativeSpeedsOutput.accept(holonomicDriveController.calculate(
                                robotPoseSupplier.get(),
                                targetPose,
                                0,
                                targetPose.getRotation()
                        )),
                        (interrupted) ->
                                robotRelativeSpeedsOutput.accept(new ChassisSpeeds()),
                        holonomicDriveController::atReference
                        );

        super.addCommands(pathFindToTargetRough);
        super.addCommands(preciseAlignment);

        super.addRequirements(driveSubsystem);
    }
}

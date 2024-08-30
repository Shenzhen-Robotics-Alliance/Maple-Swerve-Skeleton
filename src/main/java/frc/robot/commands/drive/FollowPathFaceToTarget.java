package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MapleShooterOptimization;

import java.util.Optional;
import java.util.function.Supplier;

public class FollowPathFaceToTarget {
    public static Command followPathFacetToTarget(Supplier<Translation2d> targetPositionSupplier, MapleShooterOptimization shooterOptimization, HolonomicDriveSubsystem driveSubsystem, PathPlannerPath path) {
        final Supplier<Rotation2d> rotationTargetOverride = () ->
                shooterOptimization.getShooterFacing(
                        targetPositionSupplier.get(),
                        driveSubsystem.getPose().getTranslation(),
                        driveSubsystem.getMeasuredChassisSpeedsFieldRelative()
                );
        return followPathFacetToTarget(driveSubsystem, path, rotationTargetOverride);
    }

    public static Command followPathFacetToTarget(Supplier<Translation2d> targetPositionSupplier, HolonomicDriveSubsystem driveSubsystem, PathPlannerPath path) {
        final Supplier<Rotation2d> rotationTargetOverride = () ->
                targetPositionSupplier.get().minus(driveSubsystem.getPose().getTranslation()).getAngle();
        return followPathFacetToTarget(driveSubsystem, path, rotationTargetOverride);
    }

    public static Command followPathFacetToTarget(HolonomicDriveSubsystem driveSubsystem, PathPlannerPath path, Supplier<Rotation2d> rotationTargetOverride) {
        final Command followPath = AutoBuilder.followPath(path);
        followPath.addRequirements(driveSubsystem);
        return followPath.beforeStarting(
                        () -> PPHolonomicDriveController.setRotationTargetOverride(
                                () -> Optional.of(rotationTargetOverride.get()))
                )
                .finallyDo(() -> PPHolonomicDriveController.setRotationTargetOverride(Optional::empty));
    }
}

package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.ChassisHeadingController;
import frc.robot.utils.MapleShooterOptimization;
import java.util.function.Supplier;

public class FollowPathFaceToTarget {
    public static Command followPathFacetToTarget(
            PathPlannerPath path,
            double offSetSeconds,
            Supplier<Translation2d> targetPositionSupplier,
            MapleShooterOptimization shooterOptimization) {
        final Runnable requestFaceToTarget = () -> ChassisHeadingController.getInstance()
                .setHeadingRequest(
                        new ChassisHeadingController.FaceToTargetRequest(targetPositionSupplier, shooterOptimization));
        final Runnable requestNull = () ->
                ChassisHeadingController.getInstance().setHeadingRequest(new ChassisHeadingController.NullRequest());
        return AutoBuilder.followPath(path)
                .deadlineFor(Commands.waitSeconds(offSetSeconds).andThen(requestFaceToTarget))
                .finallyDo(requestNull);
    }

    public static Command followPathFacetToTarget(
            PathPlannerPath path, double offSetSeconds, Supplier<Rotation2d> rotationTargetOverride) {
        final Runnable requestFaceToRotation = () -> ChassisHeadingController.getInstance()
                .setHeadingRequest(new ChassisHeadingController.FaceToRotationRequest(rotationTargetOverride.get()));
        final Runnable requestNull = () ->
                ChassisHeadingController.getInstance().setHeadingRequest(new ChassisHeadingController.NullRequest());
        return AutoBuilder.followPath(path)
                .deadlineFor(Commands.waitSeconds(offSetSeconds).andThen(requestFaceToRotation))
                .finallyDo(requestNull);
    }
}

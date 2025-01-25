package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.utils.ChassisHeadingController;
import frc.robot.utils.PathUtils;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Set;
import org.ironmaple.utils.FieldMirroringUtils;

public class AutoAlignment {
    private static final Pose2d ROUGH_APPROACH_TOLERANCE = new Pose2d(0.4, 0.4, Rotation2d.fromDegrees(15));

    /**
     * creates a precise auto-alignment command NOTE: AutoBuilder must be configured! the command has two steps: 1.
     * path-find to the target pose, roughly 2. accurate auto alignment
     */
    public static Command pathFindAndAutoAlign(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            Pose2d roughTarget,
            Pose2d preciseTarget,
            OptionalInt tagIdToFocus,
            Optional<Translation2d> faceToVisionTarget,
            AutoAlignmentConfigurations config) {
        Command pathFindToRoughTarget = pathFindToPose(
                        driveSubsystem,
                        roughTarget,
                        config.roughApproachSpeedFactor,
                        config.transitionSpeed,
                        faceToVisionTarget)
                .until(() -> isPoseErrorInBound(driveSubsystem.getPose(), roughTarget, ROUGH_APPROACH_TOLERANCE));
        Command preciseAlignment = preciseAlignment(driveSubsystem, roughTarget, preciseTarget, config)
                .deadlineFor(vision.focusOnTarget(tagIdToFocus));

        return pathFindToRoughTarget.andThen(preciseAlignment);
    }

    public static Command followPathAndAutoAlign(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            PathPlannerPath path,
            Pose2d preciseTargetAtBlue,
            OptionalInt tagIdToFocusAtBlue,
            OptionalInt tagIdToFocusAtRed,
            AutoAlignmentConfigurations config) {
        return Commands.deferredProxy(() -> followPathAndAutoAlignStatic(
                driveSubsystem,
                vision,
                path,
                PathUtils.getEndingPose(path),
                FieldMirroringUtils.toCurrentAlliancePose(preciseTargetAtBlue),
                FieldMirroringUtils.isSidePresentedAsRed() ? tagIdToFocusAtRed : tagIdToFocusAtBlue,
                config));
    }

    public static Command followPathAndAutoAlignStatic(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            PathPlannerPath path,
            Pose2d roughTarget,
            Pose2d preciseTarget,
            OptionalInt tagIdToFocus,
            AutoAlignmentConfigurations config) {
        Command followPath = AutoBuilder.followPath(path)
                .until(() -> isPoseErrorInBound(driveSubsystem.getPose(), roughTarget, ROUGH_APPROACH_TOLERANCE));

        Command preciseAlignment = preciseAlignment(driveSubsystem, roughTarget, preciseTarget, config)
                .deadlineFor(vision.focusOnTarget(tagIdToFocus));

        return followPath.andThen(preciseAlignment);
    }

    public static Command pathFindToPose(
            HolonomicDriveSubsystem driveSubsystem,
            Pose2d targetPose,
            double speedMultiplier,
            LinearVelocity goalEndVelocity,
            Optional<Translation2d> faceToVisionTarget) {
        ChassisHeadingController.ChassisHeadingRequest chassisHeadingRequest = faceToVisionTarget.isPresent()
                ? new ChassisHeadingController.FaceToTargetRequest(faceToVisionTarget::get, null)
                : new ChassisHeadingController.NullRequest();
        Command activateChassisHeadingController =
                Commands.runOnce(() -> ChassisHeadingController.getInstance().setHeadingRequest(chassisHeadingRequest));
        Runnable deactivateChassisHeadingController = () ->
                ChassisHeadingController.getInstance().setHeadingRequest(new ChassisHeadingController.NullRequest());
        Runnable resetDriveCommandRotationMaintenance = () -> JoystickDrive.instance.ifPresent(
                joystickDrive -> joystickDrive.setRotationMaintenanceSetpoint(targetPose.getRotation()));
        return AutoBuilder.pathfindToPose(
                        targetPose, driveSubsystem.getChassisConstrains(speedMultiplier), goalEndVelocity)
                .beforeStarting(activateChassisHeadingController)
                .finallyDo(deactivateChassisHeadingController)
                .finallyDo(resetDriveCommandRotationMaintenance);
    }

    public static Command preciseAlignment(
            HolonomicDriveSubsystem driveSubsystem,
            Pose2d roughTarget,
            Pose2d preciseTarget,
            AutoAlignmentConfigurations config) {
        return Commands.defer(
                        () -> AutoBuilder.followPath(getPreciseAlignmentPath(
                                driveSubsystem.getPose(),
                                roughTarget,
                                preciseTarget,
                                driveSubsystem.getMeasuredChassisSpeedsFieldRelative(),
                                config)),
                        Set.of(driveSubsystem))
                .beforeStarting(Commands.runOnce(RobotState.getInstance()::mergeVisionOdometryToPrimaryOdometry));
    }

    private static PathPlannerPath getPreciseAlignmentPath(
            Pose2d currentRobotPose,
            Pose2d roughTarget,
            Pose2d preciseTarget,
            ChassisSpeeds currentSpeedsFieldRelative,
            AutoAlignmentConfigurations config) {
        Translation2d currentTranslationalSpeedsMPS = new Translation2d(
                currentSpeedsFieldRelative.vxMetersPerSecond, currentSpeedsFieldRelative.vyMetersPerSecond);
        Translation2d deltaPosition = preciseTarget.getTranslation().minus(roughTarget.getTranslation());
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentRobotPose.getTranslation(), currentTranslationalSpeedsMPS.getAngle()),
                new Pose2d(preciseTarget.getTranslation(), deltaPosition.getAngle()));

        PathConstraints constraints = new PathConstraints(
                config.preciseAlignmentSpeed,
                config.preciseAlignmentMaxAcceleration,
                DegreesPerSecond.of(180),
                DegreesPerSecondPerSecond.of(360));

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(config.transitionSpeed, currentRobotPose.getRotation()),
                new GoalEndState(0.0, preciseTarget.getRotation()));
        path.preventFlipping = true;

        return path;
    }

    public static boolean isPoseErrorInBound(Pose2d currentPose, Pose2d desiredPose, Pose2d tolerance) {
        Transform2d difference = desiredPose.minus(currentPose);
        return Math.abs(difference.getX()) < tolerance.getX()
                && Math.abs(difference.getY()) < tolerance.getY()
                && Math.abs(difference.getRotation().getRadians())
                        < tolerance.getRotation().getRadians();
    }

    public record AutoAlignmentConfigurations(
            double roughApproachSpeedFactor,
            LinearVelocity transitionSpeed,
            LinearVelocity preciseAlignmentSpeed,
            LinearAcceleration preciseAlignmentMaxAcceleration) {
        public static final AutoAlignmentConfigurations DEFAULT_CONFIG = new AutoAlignmentConfigurations(
                0.6, MetersPerSecond.of(2), MetersPerSecond.of(2), MetersPerSecondPerSecond.of(4));
    }
}

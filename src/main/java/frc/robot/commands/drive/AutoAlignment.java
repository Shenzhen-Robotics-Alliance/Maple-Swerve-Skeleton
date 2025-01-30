package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.utils.ChassisHeadingController;
import frc.robot.utils.PathUtils;
import java.util.*;
import org.ironmaple.utils.FieldMirroringUtils;

public class AutoAlignment {
    private static final Distance ROUGH_APPROACH_TOLERANCE = Meters.of(0.65);
    private static final Distance PRECISE_APPROACH_STRAIGHT_FORWARD_DISTANCE = Meters.of(0.3);

    /**
     * creates a precise auto-alignment command NOTE: AutoBuilder must be configured! the command has two steps: 1.
     * path-find to the target pose, roughly 2. accurate auto alignment
     */
    public static Command pathFindAndAutoAlign(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            Pose2d roughTarget,
            Pose2d preciseTarget,
            Rotation2d preciseTargetApproachDirection,
            OptionalInt tagIdToFocus,
            OptionalInt cameraToFocus,
            Optional<Translation2d> faceToVisionTarget,
            AutoAlignmentConfigurations config) {
        Command pathFindToRoughTarget = pathFindToPose(driveSubsystem, roughTarget, faceToVisionTarget, config)
                .onlyIf(() -> RobotState.getInstance()
                                .getVisionPose()
                                .minus(preciseTarget)
                                .getTranslation()
                                .getNorm()
                        > ROUGH_APPROACH_TOLERANCE.in(Meters));
        Command preciseAlignment = preciseAlignment(
                        driveSubsystem, preciseTarget, preciseTargetApproachDirection, config)
                .deadlineFor(vision.focusOnTarget(tagIdToFocus, cameraToFocus));

        return pathFindToRoughTarget.andThen(preciseAlignment);
    }

    public static Command followPathAndAutoAlign(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            PathPlannerPath path,
            Pose2d preciseTargetAtBlue,
            Rotation2d preciseTargetApproachDirection,
            OptionalInt tagIdToFocusAtBlue,
            OptionalInt tagIdToFocusAtRed,
            OptionalInt cameraIdToFocus,
            AutoAlignmentConfigurations config) {
        return Commands.deferredProxy(() -> followPathAndAutoAlignStatic(
                driveSubsystem,
                vision,
                path,
                PathUtils.getEndingPose(path),
                FieldMirroringUtils.toCurrentAlliancePose(preciseTargetAtBlue),
                preciseTargetApproachDirection,
                FieldMirroringUtils.isSidePresentedAsRed() ? tagIdToFocusAtRed : tagIdToFocusAtBlue,
                cameraIdToFocus,
                config));
    }

    public static Command followPathAndAutoAlignStatic(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            PathPlannerPath path,
            Pose2d roughTarget,
            Pose2d preciseTarget,
            Rotation2d preciseTargetApproachDirection,
            OptionalInt tagIdToFocus,
            OptionalInt cameraToFocus,
            AutoAlignmentConfigurations config) {
        Command followPath = AutoBuilder.followPath(path)
                .until(() -> RobotState.getInstance()
                                .getVisionPose()
                                .getTranslation()
                                .minus(roughTarget.getTranslation())
                                .getNorm()
                        < ROUGH_APPROACH_TOLERANCE.in(Meters));

        Command preciseAlignment = preciseAlignment(
                        driveSubsystem, preciseTarget, preciseTargetApproachDirection, config)
                .deadlineFor(vision.focusOnTarget(tagIdToFocus, cameraToFocus));

        return followPath.andThen(preciseAlignment);
    }

    public static Command pathFindToPose(
            HolonomicDriveSubsystem driveSubsystem,
            Pose2d targetPose,
            Optional<Translation2d> faceToVisionTarget,
            AutoAlignmentConfigurations config) {
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
                        targetPose,
                        driveSubsystem.getChassisConstrains(config.roughApproachSpeedFactor),
                        config.transitionSpeed)
                .beforeStarting(activateChassisHeadingController)
                .until(() -> RobotState.getInstance()
                                .getVisionPose()
                                .getTranslation()
                                .minus(targetPose.getTranslation())
                                .getNorm()
                        < ROUGH_APPROACH_TOLERANCE.in(Meters))
                .finallyDo(deactivateChassisHeadingController)
                .finallyDo(resetDriveCommandRotationMaintenance);
    }

    public static Command preciseAlignment(
            HolonomicDriveSubsystem driveSubsystem,
            Pose2d preciseTarget,
            Rotation2d preciseTargetApproachDirection,
            AutoAlignmentConfigurations config) {
        return Commands.defer(
                        () -> AutoBuilder.followPath(getPreciseAlignmentPath(
                                driveSubsystem.getPose(), preciseTarget, preciseTargetApproachDirection, config)),
                        Set.of(driveSubsystem))
                .beforeStarting(Commands.runOnce(RobotState.getInstance()::mergeVisionOdometryToPrimaryOdometry))
                .deadlineFor(Commands.startEnd(
                        () -> RobotState.getInstance().setVisionSensitiveMode(true),
                        () -> RobotState.getInstance().setVisionSensitiveMode(false)));
    }

    private static PathPlannerPath getPreciseAlignmentPath(
            Pose2d currentRobotPose,
            Pose2d preciseTarget,
            Rotation2d preciseTargetApproachDirection,
            AutoAlignmentConfigurations config) {
        Translation2d interiorWaypoint = preciseTarget
                .getTranslation()
                .plus(new Translation2d(
                        -PRECISE_APPROACH_STRAIGHT_FORWARD_DISTANCE.in(Meters), preciseTargetApproachDirection));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(
                        currentRobotPose.getTranslation(),
                        interiorWaypoint
                                .minus(currentRobotPose.getTranslation())
                                .getAngle()),
                new Pose2d(interiorWaypoint, preciseTargetApproachDirection),
                new Pose2d(preciseTarget.getTranslation(), preciseTargetApproachDirection));

        PathConstraints globalConstrains = new PathConstraints(
                config.preciseAlignmentSpeed,
                config.preciseAlignmentMaxAcceleration,
                DegreesPerSecond.of(180),
                DegreesPerSecondPerSecond.of(360));

        PathConstraints slowDownConstrains = new PathConstraints(
                config.preciseAlignmentSpeed.times(0.4),
                config.preciseAlignmentMaxAcceleration,
                DegreesPerSecond.of(90),
                DegreesPerSecondPerSecond.of(360));

        List<RotationTarget> rotationTargets = List.of(new RotationTarget(1.0, preciseTarget.getRotation()));
        List<ConstraintsZone> constraintsZones = List.of(new ConstraintsZone(1.0, 2.0, slowDownConstrains));

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                rotationTargets,
                List.of(),
                constraintsZones,
                List.of(),
                globalConstrains,
                new IdealStartingState(config.transitionSpeed, currentRobotPose.getRotation()),
                new GoalEndState(0.0, preciseTarget.getRotation()),
                false);
        path.preventFlipping = true;

        return path;
    }

    public record AutoAlignmentConfigurations(
            double roughApproachSpeedFactor,
            LinearVelocity transitionSpeed,
            LinearVelocity preciseAlignmentSpeed,
            LinearVelocity hitTargetSpeed,
            LinearAcceleration preciseAlignmentMaxAcceleration) {
        public static final AutoAlignmentConfigurations DEFAULT_CONFIG = new AutoAlignmentConfigurations(
                0.6,
                MetersPerSecond.of(2),
                MetersPerSecond.of(2),
                MetersPerSecond.of(0.5),
                MetersPerSecondPerSecond.of(4));
    }
}

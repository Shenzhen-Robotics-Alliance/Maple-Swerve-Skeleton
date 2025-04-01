package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveControlLoops.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.constants.DriveControlLoops;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.led.LEDAnimation;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.utils.ChassisHeadingController;
import java.util.*;
import java.util.function.Supplier;

public class AutoAlignment {
    public record AutoAlignmentTarget(
            Pose2d roughTarget,
            Pose2d preciseTarget,
            Rotation2d preciseApproachDirection,
            Optional<Translation2d> faceToTargetDuringRoughApproach,
            OptionalInt tagIdToFocus,
            Integer... cameraToFocus) {}
    /**
     * creates a precise auto-alignment command NOTE: AutoBuilder must be configured! the command has two steps: 1.
     * path-find to the target pose, roughly 2. accurate auto alignment
     */
    public static Command pathFindAndAutoAlignStatic(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            LEDStatusLight statusLight,
            AutoAlignmentTarget target,
            AutoAlignmentConfigurations config,
            Command... toScheduleAtFinalApproach) {
        Command pathFindToRoughTarget = pathFindToPose(
                        target.roughTarget(), target.faceToTargetDuringRoughApproach(), config)
                .onlyIf(() -> RobotState.getInstance()
                                .getVisionPose()
                                .minus(target.roughTarget())
                                .getTranslation()
                                .getNorm()
                        > config.distanceStartPreciseApproach.in(Meters))
                .onlyIf(() -> RobotState.getInstance()
                                .getVisionPose()
                                .minus(target.preciseTarget())
                                .getTranslation()
                                .getNorm()
                        > target.roughTarget()
                                .minus(target.preciseTarget())
                                .getTranslation()
                                .getNorm());
        Command preciseAlignment = preciseAlignment(
                        driveSubsystem,
                        statusLight,
                        target.preciseTarget(),
                        target.preciseApproachDirection(),
                        config,
                        toScheduleAtFinalApproach)
                .deadlineFor(vision.focusOnTarget(target.tagIdToFocus(), target.cameraToFocus()));

        return pathFindToRoughTarget
                .andThen(preciseAlignment)
                .beforeStarting(autoAlignmentLight(statusLight)::schedule)
                .finallyDo(alignmentComplete(target::preciseTarget, statusLight)::schedule)
                .withName("Path Find & Auto Align")
                .deadlineFor(Commands.print("aligning...").repeatedly());
    }

    public static Command followPathAndAutoAlignStatic(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            LEDStatusLight statusLight,
            PathPlannerPath path,
            AutoAlignmentTarget target,
            AutoAlignmentConfigurations config,
            Command... toScheduleAtFinalApproach) {
        Command followPath = AutoBuilder.followPath(path)
                .until(() -> RobotState.getInstance()
                                .getVisionPose()
                                .getTranslation()
                                .minus(target.roughTarget().getTranslation())
                                .getNorm()
                        < config.distanceStartPreciseApproach.in(Meters));

        Command preciseAlignment = preciseAlignment(
                        driveSubsystem,
                        statusLight,
                        target.preciseTarget(),
                        target.preciseApproachDirection(),
                        config,
                        toScheduleAtFinalApproach)
                .deadlineFor(vision.focusOnTarget(target.tagIdToFocus(), target.cameraToFocus()))
                .finallyDo(driveSubsystem::stop)
                .withName("Follow Path & Auto Align");

        return followPath
                .andThen(preciseAlignment)
                .finallyDo(alignmentComplete(target::preciseTarget, statusLight)::schedule);
    }

    public static Command pathFindToPose(
            Pose2d targetPose, Optional<Translation2d> faceToVisionTarget, AutoAlignmentConfigurations config) {
        ChassisHeadingController.ChassisHeadingRequest chassisHeadingRequest = faceToVisionTarget.isPresent()
                ? new ChassisHeadingController.FaceToTargetRequest(faceToVisionTarget::get, null)
                : new ChassisHeadingController.NullRequest();
        Command activateChassisHeadingController =
                Commands.runOnce(() -> ChassisHeadingController.getInstance().setHeadingRequest(chassisHeadingRequest));
        Runnable deactivateChassisHeadingController = () ->
                ChassisHeadingController.getInstance().setHeadingRequest(new ChassisHeadingController.NullRequest());

        PathConstraints normalConstraints = new PathConstraints(
                AUTO_ALIGNMENT_VELOCITY_LIMIT,
                AUTO_ALIGNMENT_ACCELERATION_LIMIT,
                ANGULAR_VELOCITY_SOFT_CONSTRAIN,
                ANGULAR_ACCELERATION_SOFT_CONSTRAIN);
        PathConstraints lowSpeedConstrain = new PathConstraints(
                MOVEMENT_VELOCITY_SOFT_CONSTRAIN_LOW,
                ACCELERATION_SOFT_CONSTRAIN_LOW,
                ANGULAR_VELOCITY_SOFT_CONSTRAIN,
                ANGULAR_ACCELERATION_SOFT_CONSTRAIN);
        Command pathFindToPoseNormalConstrains = AutoBuilder.pathfindToPose(targetPose, normalConstraints)
                .onlyIf(() -> !RobotState.getInstance().lowSpeedModeEnabled())
                .until(RobotState.getInstance()::lowSpeedModeEnabled);
        Command pathFindToPoseLowConstrains = AutoBuilder.pathfindToPose(targetPose, lowSpeedConstrain)
                .onlyIf(RobotState.getInstance()::lowSpeedModeEnabled);
        Command pathFindToPose = pathFindToPoseNormalConstrains.andThen(pathFindToPoseLowConstrains);

        Runnable resetDriveCommandRotationMaintenance =
                () -> JoystickDrive.instance.ifPresent(joystickDrive -> joystickDrive.setRotationMaintenanceSetpoint(
                        RobotState.getInstance().getRotation()));

        return pathFindToPose
                .beforeStarting(activateChassisHeadingController)
                .until(() -> RobotState.getInstance()
                                .getVisionPose()
                                .getTranslation()
                                .minus(targetPose.getTranslation())
                                .getNorm()
                        < config.distanceStartPreciseApproach.in(Meters))
                .finallyDo(deactivateChassisHeadingController)
                .finallyDo(resetDriveCommandRotationMaintenance);
    }

    public static Command preciseAlignment(
            HolonomicDriveSubsystem driveSubsystem,
            LEDStatusLight statusLight,
            Pose2d preciseTarget,
            Rotation2d preciseTargetApproachDirection,
            AutoAlignmentConfigurations config,
            Command... toScheduleAtFinalApproach) {
        Command[] toSchedule = Arrays.copyOf(toScheduleAtFinalApproach, toScheduleAtFinalApproach.length + 1);
        toSchedule[toScheduleAtFinalApproach.length] = finalApproachLight(statusLight);
        return driveSubsystem
                .defer(() -> AutoBuilder.followPath(getPreciseAlignmentPath(
                        driveSubsystem.getMeasuredChassisSpeedsFieldRelative(),
                        driveSubsystem.getPose(),
                        preciseTarget,
                        preciseTargetApproachDirection,
                        config,
                        toSchedule)))
                .deadlineFor(RobotState.getInstance().withNavigationMode(RobotState.NavigationMode.VISION_GUIDED));
    }

    public static PathPlannerPath getPreciseAlignmentPath(
            ChassisSpeeds measuredSpeedsFieldRelative,
            Pose2d currentRobotPose,
            Pose2d preciseTarget,
            Rotation2d preciseTargetApproachDirection,
            AutoAlignmentConfigurations config,
            Command... toScheduleAtFinalApproach) {
        Translation2d interiorWaypoint = preciseTarget
                .getTranslation()
                .plus(new Translation2d(
                        -config.finalApproachStraightTrajectoryLength.in(Meters), preciseTargetApproachDirection));
        Translation2d fieldRelativeSpeedsMPS = new Translation2d(
                measuredSpeedsFieldRelative.vxMetersPerSecond, measuredSpeedsFieldRelative.vyMetersPerSecond);
        Rotation2d startingPathDirection = interiorWaypoint
                .minus(currentRobotPose.getTranslation())
                .times(AUTO_ALIGNMENT_TRANSITION_COMPENSATION_FACTOR)
                .plus(fieldRelativeSpeedsMPS)
                .getAngle();

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentRobotPose.getTranslation(), startingPathDirection),
                new Pose2d(interiorWaypoint, preciseTargetApproachDirection),
                new Pose2d(preciseTarget.getTranslation(), preciseTargetApproachDirection));

        PathConstraints constraints = new PathConstraints(
                config.finalAlignmentSpeed(),
                config.preciseAlignmentMaxAcceleration(),
                ANGULAR_VELOCITY_SOFT_CONSTRAIN_LOW,
                ANGULAR_ACCELERATION_SOFT_CONSTRAIN_LOW);

        List<RotationTarget> rotationTargets = List.of(new RotationTarget(1.0, preciseTarget.getRotation()));

        List<EventMarker> events = new ArrayList<>();
        for (Command toSchedule : toScheduleAtFinalApproach)
            events.add(new EventMarker("Final Approach", 1.0, Commands.runOnce(toSchedule::schedule)));
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                rotationTargets,
                List.of(),
                List.of(),
                events,
                constraints,
                new IdealStartingState(fieldRelativeSpeedsMPS.getNorm(), currentRobotPose.getRotation()),
                new GoalEndState(config.hitTargetSpeed, preciseTarget.getRotation()),
                false);
        path.preventFlipping = true;

        return path;
    }

    private static Command alignmentComplete(Supplier<Pose2d> goalPose, LEDStatusLight statusLight) {
        return Commands.either(
                statusLight.playAnimation(new LEDAnimation.ShowColor(() -> Color.kGreen), 0.5),
                statusLight.playAnimation(new LEDAnimation.ShowColor(() -> Color.kRed), 0.5),
                () -> {
                    Twist2d error = RobotState.getInstance().getVisionPose().log(goalPose.get());
                    System.out.println("alignment error: " + error);
                    return Math.abs(error.dtheta)
                                    <= DriveControlLoops.AUTO_ALIGNMENT_SUCCESS_TOLERANCE_ROTATIONAL.in(Radians)
                            && Math.abs(error.dy) <= AUTO_ALIGNMENT_SUCCESS_BIAS_TOLERANCE.in(Meters)
                            && Math.abs(error.dx) <= AUTO_ALIGNMENT_SUCCESS_DISTANCE_TOLERANCE.in(Meters);
                });
    }

    private static Command autoAlignmentLight(LEDStatusLight statusLight) {
        return statusLight
                .playAnimation(new LEDAnimation.Rainbow(), 1)
                .repeatedly()
                .asProxy();
    }

    private static Command finalApproachLight(LEDStatusLight statusLight) {
        return statusLight
                .playAnimationPeriodically(new LEDAnimation.Charging(Color.kHotPink), 2.0)
                .asProxy();
    }

    public record AutoAlignmentConfigurations(
            Distance distanceStartPreciseApproach,
            LinearVelocity finalAlignmentSpeed,
            Distance finalApproachStraightTrajectoryLength,
            LinearVelocity hitTargetSpeed,
            LinearAcceleration preciseAlignmentMaxAcceleration) {
        public static final AutoAlignmentConfigurations DEFAULT_CONFIG = new AutoAlignmentConfigurations(
                Meters.of(0.5),
                MetersPerSecond.of(2),
                Meters.of(0.4),
                MetersPerSecond.of(0.5),
                MetersPerSecondPerSecond.of(4));
    }
}

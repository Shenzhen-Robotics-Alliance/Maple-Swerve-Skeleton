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
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.utils.ChassisHeadingController;
import java.util.List;
import java.util.OptionalInt;
import java.util.Set;
import java.util.function.Supplier;
import org.ironmaple.utils.FieldMirroringUtils;

public class AutoAlignment {
    private static final Pose2d ROUGH_APPROACH_TOLERANCE = new Pose2d(0.4, 0.4, Rotation2d.fromDegrees(15));

    private static final LinearVelocity TRANSITION_SPEED = MetersPerSecond.of(0.4);
    private static final LinearVelocity PRECISE_ALIGNMENT_MAX_VELOCITY = MetersPerSecond.of(0.8);
    private static final LinearAcceleration PRECISE_ALIGNMENT_MAX_ACCELERATION = MetersPerSecondPerSecond.of(1);

    public static Command pathFindAndAutoAlign(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            Supplier<Pose2d> roughTarget,
            Supplier<Pose2d> preciseTarget,
            Supplier<OptionalInt> tagIdToFocus,
            double pathFindingSpeedRatio) {
        return pathFindAndAutoAlign(
                driveSubsystem,
                vision,
                roughTarget,
                preciseTarget,
                tagIdToFocus,
                pathFindingSpeedRatio,
                Commands.none());
    }

    /**
     * creates a precise auto-alignment command NOTE: AutoBuilder must be configured! the command has two steps: 1.
     * path-find to the target pose, roughly 2. accurate auto alignment
     */
    public static Command pathFindAndAutoAlign(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            Supplier<Pose2d> roughTarget,
            Supplier<Pose2d> preciseTarget,
            Supplier<OptionalInt> tagIdToFocus,
            double pathFindingSpeedRatio,
            Command toScheduleAtStartOfPreciseAlignment) {
        Command pathFindToRoughTarget = pathFindToPose(
                        driveSubsystem, roughTarget, pathFindingSpeedRatio, TRANSITION_SPEED)
                .until(() -> isPoseErrorInBound(driveSubsystem.getPose(), roughTarget.get(), ROUGH_APPROACH_TOLERANCE));
        Command preciseAlignment = preciseAlignment(driveSubsystem, roughTarget, preciseTarget);
        preciseAlignment = preciseAlignment.deadlineFor(vision.focusOnTarget(tagIdToFocus));
        preciseAlignment = preciseAlignment.beforeStarting(toScheduleAtStartOfPreciseAlignment::schedule);

        return pathFindToRoughTarget.andThen(preciseAlignment);
    }

    public static Command followPathAndAutoAlign(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            PathPlannerPath path,
            Supplier<Pose2d> preciseTarget,
            Supplier<OptionalInt> tagIdToFocus) {
        return followPathAndAutoAlign(driveSubsystem, vision, path, preciseTarget, tagIdToFocus, Commands.none());
    }

    public static Command followPathAndAutoAlign(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            PathPlannerPath path,
            Supplier<Pose2d> preciseTarget,
            Supplier<OptionalInt> tagIdToFocus,
            Command toScheduleAtStartOfPreciseAlignment) {
        Pose2d pathEndingPoseAtBlue = new Pose2d(
                path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation(),
                path.getGoalEndState().rotation());
        Command followPath = AutoBuilder.followPath(path)
                .until(() -> isPoseErrorInBound(
                        driveSubsystem.getPose(),
                        FieldMirroringUtils.toCurrentAlliancePose(pathEndingPoseAtBlue),
                        ROUGH_APPROACH_TOLERANCE));

        Command preciseAlignment = preciseAlignment(
                driveSubsystem, () -> FieldMirroringUtils.toCurrentAlliancePose(pathEndingPoseAtBlue), preciseTarget);
        preciseAlignment = preciseAlignment.deadlineFor(vision.focusOnTarget(tagIdToFocus));
        preciseAlignment = preciseAlignment.beforeStarting(toScheduleAtStartOfPreciseAlignment::schedule);

        return followPath.andThen(preciseAlignment);
    }

    public static Command pathFindToPose(
            HolonomicDriveSubsystem driveSubsystem,
            Supplier<Pose2d> targetPose,
            double speedMultiplier,
            LinearVelocity goalEndVelocity) {
        return Commands.defer(
                        () -> AutoBuilder.pathfindToPose(
                                        targetPose.get(),
                                        driveSubsystem.getChassisConstrains(speedMultiplier),
                                        goalEndVelocity)
                                .beforeStarting(Commands.runOnce(() -> ChassisHeadingController.getInstance()
                                        .setHeadingRequest(new ChassisHeadingController.NullRequest())))
                                .finallyDo(() -> ChassisHeadingController.getInstance()
                                        .setHeadingRequest(new ChassisHeadingController.NullRequest())),
                        Set.of(driveSubsystem))
                .finallyDo(() -> JoystickDrive.currentRotationMaintenanceSetpoint =
                        targetPose.get().getRotation());
    }

    public static Command preciseAlignment(
            HolonomicDriveSubsystem driveSubsystem, Supplier<Pose2d> roughTarget, Supplier<Pose2d> preciseTarget) {
        return Commands.defer(
                () -> AutoBuilder.followPath(getPreciseAlignmentPath(
                        driveSubsystem.getPose(),
                        roughTarget.get(),
                        preciseTarget.get(),
                        driveSubsystem.getMeasuredChassisSpeedsFieldRelative())),
                Set.of(driveSubsystem));
    }

    private static PathPlannerPath getPreciseAlignmentPath(
            Pose2d currentRobotPose,
            Pose2d roughTarget,
            Pose2d preciseTarget,
            ChassisSpeeds currentSpeedsFieldRelative) {
        Translation2d currentTranslationalSpeedsMPS = new Translation2d(
                currentSpeedsFieldRelative.vxMetersPerSecond, currentSpeedsFieldRelative.vyMetersPerSecond);
        Translation2d deltaPosition = preciseTarget.getTranslation().minus(roughTarget.getTranslation());
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentRobotPose.getTranslation(), currentTranslationalSpeedsMPS.getAngle()),
                new Pose2d(preciseTarget.getTranslation(), deltaPosition.getAngle()));

        PathConstraints constraints = new PathConstraints(
                PRECISE_ALIGNMENT_MAX_VELOCITY,
                PRECISE_ALIGNMENT_MAX_ACCELERATION,
                DegreesPerSecond.of(180),
                DegreesPerSecondPerSecond.of(360));

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(TRANSITION_SPEED, currentRobotPose.getRotation()),
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
}

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.utils.ChassisHeadingController;
import java.util.List;
import java.util.OptionalInt;
import java.util.Set;
import java.util.function.Supplier;

public class AutoAlignment extends SequentialCommandGroup {
    private static final Pose2d ROUGH_APPROACH_TOLERANCE = new Pose2d(0.4, 0.4, Rotation2d.fromDegrees(15));
    private Command toScheduleAtStartOfPreciseAlignment = Commands.none();

    public AutoAlignment(HolonomicDriveSubsystem driveSubsystem, AprilTagVision vision, Supplier<Pose2d> targetPose) {
        this(driveSubsystem, vision, targetPose, targetPose, OptionalInt.empty(), 0.75, MetersPerSecond.of(1));
    }

    /**
     * creates a precise auto-alignment command NOTE: AutoBuilder must be configured! the command has two steps: 1.
     * path-find to the target pose, roughly 2. accurate auto alignment
     */
    public AutoAlignment(
            HolonomicDriveSubsystem driveSubsystem,
            AprilTagVision vision,
            Supplier<Pose2d> roughTarget,
            Supplier<Pose2d> preciseTarget,
            OptionalInt tagIdToFocus,
            double speedMultiplier,
            LinearVelocity goalEndVelocityRoughApproach) {
        super.addRequirements(driveSubsystem);

        Command pathFindToTargetRough = pathFindToPose(
                        driveSubsystem, roughTarget, speedMultiplier, goalEndVelocityRoughApproach)
                .until(() -> isPoseErrorInBound(driveSubsystem.getPose(), roughTarget.get(), ROUGH_APPROACH_TOLERANCE));
        Command preciseAlignment = preciseAlignment(driveSubsystem, roughTarget, preciseTarget);
        if (tagIdToFocus.isPresent())
            preciseAlignment = preciseAlignment.deadlineFor(vision.focusOnTarget(tagIdToFocus.getAsInt()));
        preciseAlignment = preciseAlignment.beforeStarting(toScheduleAtStartOfPreciseAlignment::schedule);

        super.addCommands(pathFindToTargetRough);
        super.addCommands(preciseAlignment);
    }

    public AutoAlignment withScheduleCommandAtStartOfPreciseAlignment(Command toScheduleAtStartOfPreciseAlignment) {
        this.toScheduleAtStartOfPreciseAlignment = toScheduleAtStartOfPreciseAlignment;
        return this;
    }

    public static Command pathFindToPose(
            HolonomicDriveSubsystem driveSubsystem,
            Supplier<Pose2d> targetPose,
            double speedMultiplier,
            LinearVelocity goalEndVelocity) {
        return Commands.defer(
                () -> AutoBuilder.pathfindToPose(
                                targetPose.get(), driveSubsystem.getChassisConstrains(speedMultiplier), goalEndVelocity)
                        .beforeStarting(Commands.runOnce(() -> ChassisHeadingController.getInstance()
                                .setHeadingRequest(new ChassisHeadingController.NullRequest())))
                        .finallyDo(() -> ChassisHeadingController.getInstance()
                                .setHeadingRequest(new ChassisHeadingController.NullRequest())),
                Set.of(driveSubsystem));
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

        PathConstraints constraints = new PathConstraints(0.8, 1.0, Math.toDegrees(180), Math.toDegrees(360));

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(currentTranslationalSpeedsMPS.getNorm(), currentRobotPose.getRotation()),
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

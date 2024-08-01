package frc.robot.commands.drive;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import org.littletonrobotics.junction.Logger;

public class CustomFollowPath extends Command {
    public static final Pose2d DEFAULT_TOLERANCE = new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(5));
    private final HolonomicDriveSubsystem driveSubsystem;
    private PathPlannerPath path;
    private final String logPath;
    private final HolonomicDriveController controller;
    private final Timer pathTimer;
    private final double speedMultiplier;

    private PathPlannerTrajectory trajectory;
    public CustomFollowPath(HolonomicDriveSubsystem driveSubsystem, PathPlannerPath path, String logPath) {
        this(driveSubsystem, path, DEFAULT_TOLERANCE, 1, logPath);
    }

    public CustomFollowPath(HolonomicDriveSubsystem driveSubsystem, PathPlannerPath path, Pose2d tolerance, double speedMultiplier, String logPath) {
        super();
        checkMultiplierValidity(speedMultiplier);
        this.driveSubsystem = driveSubsystem;
        this.speedMultiplier = speedMultiplier;
        this.path = path;
        this.logPath = logPath;

        this.controller = DriveToPosition.createPositionController();
        controller.setTolerance(tolerance);
        super.addRequirements(driveSubsystem);

        this.pathTimer = new Timer();
        pathTimer.start();
    }

    public static void checkMultiplierValidity(double speedMultiplier) {
        if (speedMultiplier > 1 || speedMultiplier <= 0)
            throw new IllegalArgumentException("speed multiplier must be within the range 0 < x <=1");
    }

    @Override
    public void initialize() {
        this.trajectory = path.getTrajectory(
                driveSubsystem.getMeasuredChassisSpeedsRobotRelative(),
                driveSubsystem.getFacing()
        );
        pathTimer.reset();

        final Pose2d[] trajectoryPoses = trajectory.getStates().stream().map(
                (state) -> new Pose2d(state.positionMeters, state.heading)
        ).toArray(Pose2d[]::new);
        Logger.recordOutput(logPath+"Trajectory", trajectoryPoses);
    }

    @Override
    public void execute() {
        final PathPlannerTrajectory.State currentTrajectoryState = trajectory.sample(
                speedMultiplier * pathTimer.get()
        );
        final Pose2d currentPoseSetpoint = new Pose2d(
                currentTrajectoryState.positionMeters,
                currentTrajectoryState.targetHolonomicRotation
        );
        this.driveSubsystem.runRobotCentricChassisSpeeds(controller.calculate(
                driveSubsystem.getPose(),
                new Pose2d(currentTrajectoryState.positionMeters, currentTrajectoryState.heading),
                currentTrajectoryState.velocityMps,
                currentTrajectoryState.targetHolonomicRotation
        ));
        Logger.recordOutput(logPath+"Current Setpoint", currentPoseSetpoint);
    }

    private static Trajectory.State toWpilibTrajectoryState(PathPlannerTrajectory.State pathPlannerTrajectoryState) {
        return new Trajectory.State(
                pathPlannerTrajectoryState.timeSeconds,
                pathPlannerTrajectoryState.velocityMps,
                pathPlannerTrajectoryState.accelerationMpsSq,
                new Pose2d(
                        pathPlannerTrajectoryState.positionMeters,
                        pathPlannerTrajectoryState.targetHolonomicRotation
                ),
                pathPlannerTrajectoryState.curvatureRadPerMeter
        );
    }

    @Override
    public boolean isFinished() {
        return pathTimer.hasElapsed(trajectory.getTotalTimeSeconds()) && controller.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
        Logger.recordOutput(logPath+"Current Setpoint", new Pose2d());
        Logger.recordOutput(logPath+"Trajectory", new Pose2d[0]);
    }

    public Command rePlannedOnStart() {
        return new CustomFollowPathOnFly(
                driveSubsystem,
                () -> path,
                true,
                logPath
        );
    }
}

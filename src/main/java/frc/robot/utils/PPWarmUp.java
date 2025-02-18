package frc.robot.utils;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PPWarmUp {
    public static Command pathFindingWarmup() {
        return new PathfindingCommand(
                        new Pose2d(2, 7, Rotation2d.k180deg),
                        new PathConstraints(4, 8, 4, 8),
                        () -> new Pose2d(5, 1, Rotation2d.kZero),
                        ChassisSpeeds::new,
                        (speeds, feedforwards) -> {},
                        new PPHolonomicDriveController(
                                new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                        new RobotConfig(
                                30,
                                6.8,
                                new ModuleConfig(
                                        0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
                                0.55))
                .andThen(Commands.print("[PathPlanner] PathfindingCommand finished warmup"))
                .ignoringDisable(true);
    }

    public static Command choreoWarmUp() {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromChoreoTrajectory("place first");
        } catch (Exception e) {
            return Commands.none();
        }
        return new FollowPathCommand(
                        path,
                        () -> new Pose2d(5, 1, Rotation2d.kZero),
                        ChassisSpeeds::new,
                        (chassisSpeeds, feedforwards) -> {},
                        new PPHolonomicDriveController(
                                new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                        new RobotConfig(
                                30,
                                6.8,
                                new ModuleConfig(
                                        0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
                                0.55),
                        () -> false)
                .andThen(Commands.print("[PathPlanner] Choreo Path Follower finished warmup"))
                .ignoringDisable(true);
    }
}

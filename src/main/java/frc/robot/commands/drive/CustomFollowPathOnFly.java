package frc.robot.commands.drive;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.CommandOnFly;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;

import java.util.function.Supplier;

public class CustomFollowPathOnFly extends CommandOnFly {
    public CustomFollowPathOnFly(HolonomicDriveSubsystem driveSubsystem, Supplier<PathPlannerPath> pathSupplier, boolean rePlanOnStart, String logPath) {
        this(driveSubsystem, pathSupplier, CustomFollowPath.DEFAULT_TOLERANCE, 1, rePlanOnStart, logPath);
    }
    public CustomFollowPathOnFly(HolonomicDriveSubsystem driveSubsystem, Supplier<PathPlannerPath> pathSupplier, Pose2d tolerance, double speedMultiplier, boolean rePlanOnStart, String logPath) {
        super(() ->
                new CustomFollowPath(
                        driveSubsystem,
                        rePlanOnStart ?
                                pathSupplier.get().replan(
                                        driveSubsystem.getPose(),
                                        driveSubsystem.getMeasuredChassisSpeedsRobotRelative()
                                ) : pathSupplier.get(),
                        tolerance,
                        speedMultiplier,
                        logPath
                )
        );
        /* throw exception right now if the multiplier is invalid */
        CustomFollowPath.checkMultiplierValidity(speedMultiplier);
    }
}

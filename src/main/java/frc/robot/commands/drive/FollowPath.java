package frc.robot.commands.drive;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;

import java.util.function.BooleanSupplier;

/**
 * this replaces AutoBuilder because in our code, opponent robots are simulated using path-planner drive commands
 * only the main robot should use AutoBuilder, opponent robots should use this
 * */
public class FollowPath extends FollowPathHolonomic {
    public FollowPath(BooleanSupplier shouldFlipPath, HolonomicDriveSubsystem driveSubsystem, double speedMultiplier, Rotation2d endingRotation, Pose2d... poses) {
        this(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(poses),
                        driveSubsystem.getChassisConstrains(speedMultiplier),
                        new GoalEndState(0, endingRotation)
                ),
                shouldFlipPath,
                driveSubsystem
        );
    }

    public FollowPath(PathPlannerPath path, BooleanSupplier shouldFlipPath, HolonomicDriveSubsystem driveSubsystem) {
        super(
                path,
                driveSubsystem::getPose,
                driveSubsystem::getMeasuredChassisSpeedsRobotRelative,
                driveSubsystem::runRobotCentricChassisSpeeds,
                Constants.SwerveDriveChassisConfigs.chassisTranslationPIDConfig.toPathPlannerPIDConstants(),
                Constants.SwerveDriveChassisConfigs.chassisRotationalPIDConfig.toPathPlannerPIDConstants(),
                driveSubsystem.getChassisMaxLinearVelocityMetersPerSec(),
                driveSubsystem.getChassisMaxLinearVelocityMetersPerSec() / driveSubsystem.getChassisMaxAngularVelocity(),
                Robot.defaultPeriodSecs,
                new ReplanningConfig(),
                shouldFlipPath,
                driveSubsystem
        );
    }
}

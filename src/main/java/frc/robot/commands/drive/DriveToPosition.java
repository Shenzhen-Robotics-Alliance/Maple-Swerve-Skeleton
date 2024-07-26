package frc.robot.commands.drive;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MechanismControl.MaplePIDController;
import frc.robot.utils.MechanismControl.MapleProfiledPIDController;

import java.util.function.Supplier;

public class DriveToPosition {
    private final Supplier<Pose2d> desiredPoseSupplier;
    private final HolonomicDriveSubsystem driveSubsystem;
    private final HolonomicDriveController positionController;

    public DriveToPosition(Supplier<Pose2d> desiredPoseSupplier, HolonomicDriveSubsystem driveSubsystem) {
        this.desiredPoseSupplier = desiredPoseSupplier;
        this.driveSubsystem = driveSubsystem;

        this.positionController = new HolonomicDriveController(
                new MaplePIDController(Constants.SwerveDriveChassisConfigs.chassisTranslationPIDConfig),
                new MaplePIDController(Constants.SwerveDriveChassisConfigs.chassisTranslationPIDConfig),
                new MapleProfiledPIDController(Constants.SwerveDriveChassisConfigs.chassisRotationalPIDConfig, Constants.SwerveDriveChassisConfigs.chassisRotationalConstraints)
        );
    }
}

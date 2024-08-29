package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.Robot;
import frc.robot.constants.DriveControlLoops;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.CustomPIDs.MaplePIDController;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

import static frc.robot.subsystems.drive.HolonomicDriveSubsystem.isZero;
import static frc.robot.constants.JoystickConfigs.*;

public class JoystickDrive extends Command {
    protected MapleJoystickDriveInput input;
    private final BooleanSupplier useDriverStationCentricSwitch;
    private final HolonomicDriveSubsystem driveSubsystem;
    private final PIDController chassisRotationController;

    protected final Timer previousChassisUsageTimer, previousRotationalInputTimer;
    private ChassisSpeeds currentPilotInputSpeeds;
    protected Rotation2d currentRotationMaintenanceSetpoint;

    private double translationalSensitivity = 1, rotationalSensitivity = 0.5;
    public JoystickDrive(MapleJoystickDriveInput input, BooleanSupplier useDriverStationCentricSwitch, HolonomicDriveSubsystem driveSubsystem) {
        super();
        this.input = input;
        this.useDriverStationCentricSwitch = useDriverStationCentricSwitch;
        this.driveSubsystem = driveSubsystem;
        this.previousChassisUsageTimer = new Timer();
        this.previousChassisUsageTimer.start();
        this.previousRotationalInputTimer = new Timer();
        this.previousRotationalInputTimer.start();
        this.chassisRotationController = new MaplePIDController(DriveControlLoops.CHASSIS_ROTATION_CLOSE_LOOP);

        super.addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.previousChassisUsageTimer.reset();
        this.previousRotationalInputTimer.reset();
        this.currentPilotInputSpeeds = new ChassisSpeeds();
        this.currentRotationMaintenanceSetpoint = driveSubsystem.getRawGyroYaw();

        this.chassisRotationController.calculate(driveSubsystem.getRawGyroYaw().getRadians()); // activate controller
    }

    @Override
    public void execute() {
        if (input == null) return;

        final ChassisSpeeds newestPilotInputSpeed = input.getJoystickChassisSpeeds(
                driveSubsystem.getChassisMaxLinearVelocityMetersPerSec() * translationalSensitivity,
                driveSubsystem.getChassisMaxAngularVelocity() * rotationalSensitivity
        );
        currentPilotInputSpeeds = driveSubsystem.constrainAcceleration(
                currentPilotInputSpeeds,
                newestPilotInputSpeed,
                Robot.defaultPeriodSecs
        );
        if (!isZero(currentPilotInputSpeeds))
            previousChassisUsageTimer.reset();
        if (Math.abs(currentPilotInputSpeeds.omegaRadiansPerSecond) > 0.05)
            previousRotationalInputTimer.reset();
        Logger.recordOutput("JoystickDrive/current pilot input speeds", currentPilotInputSpeeds.toString());

        if (previousChassisUsageTimer.hasElapsed(NON_USAGE_TIME_RESET_WHEELS)) {
            driveSubsystem.stop();
            return;
        }

        if (Math.hypot(currentPilotInputSpeeds.vxMetersPerSecond, currentPilotInputSpeeds.vyMetersPerSecond) < 0.01
                && Math.abs(currentPilotInputSpeeds.omegaRadiansPerSecond) < 0.01)
            currentPilotInputSpeeds = new ChassisSpeeds();

        final ChassisSpeeds chassisSpeedsWithRotationMaintenance;
        final double rotationCorrectionAngularVelocity = chassisRotationController.calculate(
                driveSubsystem.getRawGyroYaw().getRadians(),
                currentRotationMaintenanceSetpoint.getRadians()
        );
        Logger.recordOutput("previousRotationalInputTimer.get()", previousRotationalInputTimer.get());
        if (previousRotationalInputTimer.get() > TIME_ACTIVATE_ROTATION_MAINTENANCE_AFTER_NO_ROTATIONAL_INPUT_SECONDS)
            chassisSpeedsWithRotationMaintenance = new ChassisSpeeds(
                    currentPilotInputSpeeds.vxMetersPerSecond, currentPilotInputSpeeds.vyMetersPerSecond,
                    rotationCorrectionAngularVelocity
            );
        else {
            chassisSpeedsWithRotationMaintenance = currentPilotInputSpeeds;
            Logger.recordOutput("pilot input speeds", currentPilotInputSpeeds);
            currentRotationMaintenanceSetpoint = driveSubsystem.getRawGyroYaw();
        }

        Logger.recordOutput("JoystickDrive/rotation maintenance set-point (deg)", currentRotationMaintenanceSetpoint.getDegrees());
        Logger.recordOutput("JoystickDrive/previous rotational input time", previousRotationalInputTimer.get());
        Logger.recordOutput("JoystickDrive/rotation closed loop velocity (deg per sec)", Math.toDegrees(rotationCorrectionAngularVelocity));

        if (useDriverStationCentricSwitch.getAsBoolean())
            driveSubsystem.runDriverStationCentricChassisSpeeds(chassisSpeedsWithRotationMaintenance);
        else
            driveSubsystem.runRobotCentricChassisSpeeds(chassisSpeedsWithRotationMaintenance);
    }

    public void setCurrentRotationalMaintenance(Rotation2d setPointAbsoluteFacing) {
        final Rotation2d gyroReadingBiasFromActualFacing = driveSubsystem.getRawGyroYaw().minus(driveSubsystem.getFacing());
        this.currentRotationMaintenanceSetpoint = setPointAbsoluteFacing.rotateBy(gyroReadingBiasFromActualFacing);
    }

    public void setSensitivity(double translationalSensitivity, double rotationalSensitivity) {
        this.translationalSensitivity = translationalSensitivity;
        this.rotationalSensitivity = rotationalSensitivity;
    }
}

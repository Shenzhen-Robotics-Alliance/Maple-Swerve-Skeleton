package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MapleJoystickDriveInput;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

import static frc.robot.subsystems.drive.HolonomicDriveSubsystem.isZero;

public class JoystickDrive extends Command {
    private final MapleJoystickDriveInput input;
    private final BooleanSupplier useDriverStationCentricSwitch;
    private final HolonomicDriveSubsystem driveSubsystem;

    private final Timer nonSpeedTimer;
    private ChassisSpeeds currentPilotInputSpeeds = new ChassisSpeeds();
    public JoystickDrive(MapleJoystickDriveInput input, BooleanSupplier useDriverStationCentricSwitch, HolonomicDriveSubsystem driveSubsystem) {
        super();
        this.input = input;
        this.useDriverStationCentricSwitch = useDriverStationCentricSwitch;
        this.driveSubsystem = driveSubsystem;

        this.nonSpeedTimer = new Timer();

        super.addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.nonSpeedTimer.reset();
        this.currentPilotInputSpeeds = new ChassisSpeeds();
    }

    @Override
    public void execute() {
        final ChassisSpeeds newestPilotInputSpeed = input.getJoystickChassisSpeeds(
                driveSubsystem.getChassisMaxLinearVelocity(), driveSubsystem.getChassisMaxAngularVelocity()
        );
        currentPilotInputSpeeds = driveSubsystem.constrainAcceleration(
                currentPilotInputSpeeds,
                newestPilotInputSpeed,
                Robot.defaultPeriodSecs
        );
        if (!isZero(currentPilotInputSpeeds))
            nonSpeedTimer.reset();
        Logger.recordOutput("current pilot input speeds", currentPilotInputSpeeds.toString());

        if (nonSpeedTimer.hasElapsed(Constants.DriveConfigs.nonUsageTimeResetWheels)) {
            driveSubsystem.stop();
            return;
        }

        if (Math.hypot(currentPilotInputSpeeds.vxMetersPerSecond, currentPilotInputSpeeds.vyMetersPerSecond) < 0.01
                && Math.abs(currentPilotInputSpeeds.omegaRadiansPerSecond) < 0.01)
            currentPilotInputSpeeds = new ChassisSpeeds();

        if (useDriverStationCentricSwitch.getAsBoolean())
            driveSubsystem.runDriverStationCentricChassisSpeeds(currentPilotInputSpeeds);
        else
            driveSubsystem.runRobotCentricChassisSpeeds(currentPilotInputSpeeds);
    }
}

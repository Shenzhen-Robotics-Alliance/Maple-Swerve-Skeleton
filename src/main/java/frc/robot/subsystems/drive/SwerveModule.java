// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveTrainConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.subsystems.drive.IO.ModuleIO;
import frc.robot.subsystems.drive.IO.ModuleIOInputsAutoLogged;
import frc.robot.utils.Alert;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends MapleSubsystem {
    private final ModuleIO io;
    private final String name;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private SwerveModuleState setPoint;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    private final Alert driveMotorHardwareFault, steerMotorHardwareFault, steerEncoderHardwareFault;

    public SwerveModule(ModuleIO io, String name) {
        super("Module-" + name);
        this.io = io;
        this.name = name;
        this.driveMotorHardwareFault =
                new Alert("Module-" + name + " Drive Motor Hardware Fault Detected", Alert.AlertType.ERROR);
        this.steerMotorHardwareFault =
                new Alert("Module-" + name + " Steer Motor Hardware Fault Detected", Alert.AlertType.ERROR);
        this.steerEncoderHardwareFault =
                new Alert("Module-" + name + " Steer Encoder Hardware Fault Detected", Alert.AlertType.ERROR);
        this.driveMotorHardwareFault.setActivated(false);
        this.steerMotorHardwareFault.setActivated(false);
        this.steerEncoderHardwareFault.setActivated(false);

        CommandScheduler.getInstance().unregisterSubsystem(this);

        setPoint = new SwerveModuleState();
        io.setDriveBrake(true);
        io.setSteerBrake(true);
    }

    public void updateOdometryInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module-" + name, inputs);
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        updateOdometryPositions();

        this.driveMotorHardwareFault.setActivated(!inputs.driveMotorConnected);
        this.steerMotorHardwareFault.setActivated(!inputs.steerMotorConnected);
        this.steerEncoderHardwareFault.setActivated(!inputs.steerEncoderConnected);
    }

    private void updateOdometryPositions() {
        odometryPositions = new SwerveModulePosition[inputs.odometryDriveWheelRevolutions.length];
        for (int i = 0; i < odometryPositions.length; i++) {
            double positionMeters = driveWheelRevolutionsToMeters(inputs.odometryDriveWheelRevolutions[i]);
            Rotation2d angle = inputs.odometrySteerPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    /** Runs the module with the specified setpoint state. Returns the optimized state. */
    public SwerveModuleState runSetPoint(
            SwerveModuleState newSetpoint, Force robotRelativeFeedforwardForceX, Force robotRelativeFeedforwardForceY) {
        newSetpoint = SwerveModuleState.optimize(newSetpoint, getSteerFacing());

        if (Math.abs(newSetpoint.speedMetersPerSecond) < 0.01) {
            io.stop();
            return this.setPoint = new SwerveModuleState(0, setPoint.angle);
        }

        double desiredWheelVelocityRadPerSec = newSetpoint.speedMetersPerSecond / WHEEL_RADIUS.in(Meters);
        Translation2d force2d = new Translation2d(
                robotRelativeFeedforwardForceX.in(Newtons), robotRelativeFeedforwardForceY.in(Newtons));
        // project force to swerve heading
        double moduleFeedforwardForceNewtons =
                force2d.getNorm() * force2d.getAngle().minus(getSteerFacing()).getCos();
        double wheelFeedforwardTorque = moduleFeedforwardForceNewtons * WHEEL_RADIUS.in(Meters);
        double motorFeedforwardTorque = wheelFeedforwardTorque / DRIVE_GEAR_RATIO;
        Voltage motorFeedforwardVoltage = Volts.of(DRIVE_MOTOR.getVoltage(motorFeedforwardTorque, 0));
        io.requestDriveVelocityControl(desiredWheelVelocityRadPerSec, motorFeedforwardVoltage);
        Logger.recordOutput("ModuleFeedforwards/" + name + "/Voltage", motorFeedforwardVoltage.in(Volts));
        io.requestSteerPositionControl(newSetpoint.angle);

        return this.setPoint = newSetpoint;
    }

    @Override
    public void onDisable() {
        io.stop();
    }

    public void setMotorBrake(boolean motorBrakeEnabled) {
        io.setDriveBrake(motorBrakeEnabled);
        io.setSteerBrake(motorBrakeEnabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getSteerFacing() {
        return inputs.steerFacing;
    }

    public double getSteerVelocityRadPerSec() {
        return inputs.steerVelocityRadPerSec;
    }

    /** Returns the current drive position of the module in meters. */
    public double getDrivePositionMeters() {
        return driveWheelRevolutionsToMeters(inputs.driveWheelFinalRevolutions);
    }

    private double driveWheelRevolutionsToMeters(double driveWheelRevolutions) {
        return Units.rotationsToRadians(driveWheelRevolutions) * WHEEL_RADIUS.in(Meters);
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getDriveVelocityMetersPerSec() {
        return driveWheelRevolutionsToMeters(inputs.driveWheelFinalVelocityRevolutionsPerSec);
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getLatestPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getSteerFacing());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getMeasuredState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSec(), getSteerFacing());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public boolean hasHardwareFaults() {
        return !(inputs.driveMotorConnected && inputs.steerMotorConnected && inputs.steerEncoderConnected);
    }
}

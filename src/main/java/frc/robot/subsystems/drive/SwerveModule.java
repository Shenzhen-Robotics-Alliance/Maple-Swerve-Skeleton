// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveControlLoops.SWERVE_VELOCITY_DEADBAND;
import static frc.robot.constants.DriveControlLoops.USE_TORQUE_FEEDFORWARD;
import static frc.robot.constants.DriveTrainConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.IO.ModuleIO;
import frc.robot.subsystems.drive.IO.ModuleIOInputsAutoLogged;
import frc.robot.utils.AlertsManager;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final ModuleIO io;
    private final String name;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private SwerveModuleState setPoint;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    private final Alert driveMotorHardwareFault, steerMotorHardwareFault, steerEncoderHardwareFault;

    public SwerveModule(ModuleIO io, String name) {
        this.io = io;
        this.name = name;
        this.driveMotorHardwareFault =
                AlertsManager.create("Module-" + name + " Drive Motor Hardware Fault Detected", Alert.AlertType.kError);
        this.steerMotorHardwareFault =
                AlertsManager.create("Module-" + name + " Steer Motor Hardware Fault Detected", Alert.AlertType.kError);
        this.steerEncoderHardwareFault = AlertsManager.create(
                "Module-" + name + " Steer Encoder Hardware Fault Detected", Alert.AlertType.kError);
        this.driveMotorHardwareFault.set(false);
        this.steerMotorHardwareFault.set(false);
        this.steerEncoderHardwareFault.set(false);

        setPoint = new SwerveModuleState();
        io.setDriveBrake(true);
        io.setSteerBrake(true);
    }

    public void updateOdometryInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module-" + name, inputs);
    }

    public void modulePeriodic() {
        updateOdometryPositions();
        if (DriverStation.isDisabled()) stop();

        this.driveMotorHardwareFault.set(!inputs.driveMotorConnected);
        this.steerMotorHardwareFault.set(!inputs.steerMotorConnected);
        this.steerEncoderHardwareFault.set(!inputs.steerEncoderConnected);
    }

    private void updateOdometryPositions() {
        odometryPositions = new SwerveModulePosition[inputs.odometryDriveWheelRevolutions.length];
        for (int i = 0; i < odometryPositions.length; i++) {
            double positionMeters = driveWheelRevolutionsToMeters(inputs.odometryDriveWheelRevolutions[i]);
            Rotation2d angle = inputs.odometrySteerPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    public SwerveModuleState runSetPoint(
            SwerveModuleState newSetpoint, Force robotRelativeFeedforwardForceX, Force robotRelativeFeedforwardForceY) {
        if (Math.abs(newSetpoint.speedMetersPerSecond) < SWERVE_VELOCITY_DEADBAND.in(MetersPerSecond)) {
            stop();
            return this.setPoint = new SwerveModuleState(0, setPoint.angle);
        }

        return forceRunSetPoint(newSetpoint, robotRelativeFeedforwardForceX, robotRelativeFeedforwardForceY);
    }

    /** Runs the module with the specified setpoint state. Returns the optimized state. */
    public SwerveModuleState forceRunSetPoint(
            SwerveModuleState newSetpoint, Force robotRelativeFeedforwardForceX, Force robotRelativeFeedforwardForceY) {
        newSetpoint = SwerveModuleState.optimize(newSetpoint, getSteerFacing());

        double desiredWheelVelocityRadPerSec = newSetpoint.speedMetersPerSecond / WHEEL_RADIUS.in(Meters);
        Translation2d force2d = new Translation2d(
                robotRelativeFeedforwardForceX.in(Newtons), robotRelativeFeedforwardForceY.in(Newtons));
        // project force to swerve heading
        double moduleFeedforwardForceNewtons =
                force2d.getNorm() * force2d.getAngle().minus(getSteerFacing()).getCos();
        double wheelFeedforwardTorque = moduleFeedforwardForceNewtons * WHEEL_RADIUS.in(Meters);
        double motorFeedforwardTorque = wheelFeedforwardTorque / DRIVE_GEAR_RATIO;
        Voltage motorFeedforwardVoltage = Volts.of(DRIVE_MOTOR.getVoltage(motorFeedforwardTorque, 0));
        if (!USE_TORQUE_FEEDFORWARD) motorFeedforwardVoltage = Volts.zero();
        io.requestDriveVelocityControl(desiredWheelVelocityRadPerSec, motorFeedforwardVoltage);
        Logger.recordOutput("ModuleFeedforwards/" + name + "/Voltage", motorFeedforwardVoltage.in(Volts));
        io.requestSteerPositionControl(newSetpoint.angle);

        return this.setPoint = newSetpoint;
    }

    public void stop() {
        io.requestDriveOpenLoop(Volts.zero());
        io.requestSteerOpenLoop(Volts.zero());
    }

    private boolean brakeEnabled = true;

    public void setMotorBrake(boolean enableMotorBrake) {
        if (brakeEnabled == enableMotorBrake) return;
        io.setDriveBrake(enableMotorBrake);
        io.setSteerBrake(enableMotorBrake);
        this.brakeEnabled = enableMotorBrake;
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

    public void runVoltageCharacterization(Rotation2d steerFacing, Voltage driveVoltageOut) {
        setMotorBrake(true);
        io.requestSteerPositionControl(steerFacing);
        io.requestDriveOpenLoop(driveVoltageOut);
    }

    public void runCurrentCharacterization(Rotation2d steerFacing, Current driveCurrentOut) {
        setMotorBrake(true);
        io.requestSteerPositionControl(steerFacing);
        io.requestDriveOpenLoop(driveCurrentOut);
    }
}

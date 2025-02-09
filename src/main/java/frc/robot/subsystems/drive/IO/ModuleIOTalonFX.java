// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Modified by 5516 Iron Maple for maple-sim
// https://github.com/Shenzhen-Robotics-Alliance/maple-sim
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveTrainConstants.*;
import static frc.robot.utils.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.generated.TunerConstants;

public abstract class ModuleIOTalonFX implements ModuleIO {
    protected final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    protected final TalonFX driveTalon;
    protected final TalonFX steerTalon;
    protected final CANcoder cancoder;

    protected final VoltageOut voltageRequest = new VoltageOut(0);
    protected final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    protected final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    protected final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
    protected final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

    // Inputs from drive motor
    protected final StatusSignal<Angle> drivePosition;
    protected final StatusSignal<AngularVelocity> driveVelocity;
    protected final StatusSignal<Voltage> driveAppliedVolts;
    protected final StatusSignal<Current> driveStatorCurrent;
    protected final StatusSignal<Current> driveSupplyCurrent;

    // Inputs from turn motor
    protected final StatusSignal<Angle> steerAbsolutePosition;
    protected final StatusSignal<AngularVelocity> steerVelocity;
    protected final StatusSignal<Voltage> steerMotorAppliedVolts;
    protected final StatusSignal<Current> steerMotorStatorCurrent;

    // Connection debouncers
    private final Debouncer driveMotorConnectedDebounce = new Debouncer(0.5);
    private final Debouncer steerMotorConnectedDebounce = new Debouncer(0.5);
    private final Debouncer steerEncoderConnectedDebounce = new Debouncer(0.5);

    protected ModuleIOTalonFX(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;

        driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        steerTalon = new TalonFX(constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        cancoder = new CANcoder(constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

        // Configure drive motor
        var driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig
                .TorqueCurrent
                .withPeakForwardTorqueCurrent(DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT)
                .withPeakReverseTorqueCurrent(DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT);
        driveConfig
                .CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(DRIVE_OVER_CURRENT_PROTECTION)
                .withSupplyCurrentLowerTime(DRIVE_OVERHEAT_PROTECTION_TIME)
                .withSupplyCurrentLowerLimit(DRIVE_OVERHEAT_PROTECTION_CURRENT_LIMIT);
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        var steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.Slot0 = constants.SteerMotorGains;

        steerConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        steerConfig.Feedback.FeedbackSensorSource = switch (constants.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;};
        steerConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfig
                .TorqueCurrent
                .withPeakForwardTorqueCurrent(STEER_CURRENT_LIMIT)
                .withPeakReverseTorqueCurrent(STEER_CURRENT_LIMIT);
        steerConfig.CurrentLimits.withStatorCurrentLimitEnable(true).withStatorCurrentLimit(STEER_CURRENT_LIMIT);
        steerConfig.MotorOutput.Inverted = constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> steerTalon.getConfigurator().apply(steerConfig, 0.25));

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveStatorCurrent = driveTalon.getStatorCurrent();
        driveSupplyCurrent = driveTalon.getSupplyCurrent();

        // Create turn status signals
        steerAbsolutePosition = cancoder.getAbsolutePosition();
        steerVelocity = steerTalon.getVelocity();
        steerMotorAppliedVolts = steerTalon.getMotorVoltage();
        steerMotorStatorCurrent = steerTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_FREQUENCY, steerAbsolutePosition, drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveStatorCurrent,
                steerVelocity,
                steerMotorAppliedVolts,
                steerMotorStatorCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, steerTalon);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus =
                BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveStatorCurrent);
        var turnStatus = BaseStatusSignal.refreshAll(steerVelocity, steerMotorAppliedVolts, steerMotorStatorCurrent);
        var turnEncoderStatus = BaseStatusSignal.refreshAll(steerAbsolutePosition);

        // Update drive inputs
        inputs.driveMotorConnected = driveMotorConnectedDebounce.calculate(driveStatus.isOK());
        inputs.driveWheelFinalRevolutions = drivePosition.getValueAsDouble() / constants.DriveMotorGearRatio;
        inputs.driveWheelFinalVelocityRevolutionsPerSec =
                driveVelocity.getValueAsDouble() / constants.DriveMotorGearRatio;
        inputs.driveMotorAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveMotorCurrentAmps = driveStatorCurrent.getValueAsDouble();

        // Update turn inputs
        inputs.steerMotorConnected = steerMotorConnectedDebounce.calculate(turnStatus.isOK());
        inputs.steerEncoderConnected = steerEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.steerFacing = Rotation2d.fromRotations(steerAbsolutePosition.getValueAsDouble());
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
        inputs.steerMotorAppliedVolts = steerMotorAppliedVolts.getValueAsDouble();
        inputs.steerMotorCurrentAmps = steerMotorStatorCurrent.getValueAsDouble();
    }

    @Override
    public void requestDriveOpenLoop(Voltage output) {
        driveTalon.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void requestDriveOpenLoop(Current output) {
        driveTalon.setControl(torqueCurrentRequest.withOutput(output));
    }

    @Override
    public void requestSteerOpenLoop(Voltage output) {
        steerTalon.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void requestSteerOpenLoop(Current output) {
        steerTalon.setControl(torqueCurrentRequest.withOutput(output));
    }

    @Override
    public void requestDriveVelocityControl(
            AngularVelocity desiredWheelVelocity, Torque additionalFeedforwardWheelTorque) {
        double desiredMotorVelocityRPS =
                desiredWheelVelocity.times(constants.DriveMotorGearRatio).in(RotationsPerSecond);
        double additionalFeedforwardMotorTorqueNM = additionalFeedforwardWheelTorque
                .div(constants.DriveMotorGearRatio)
                .in(NewtonMeters);
        double measuredMotorVelocityRadPerSec = driveVelocity.getValue().in(RadiansPerSecond);
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> velocityVoltageRequest
                            .withVelocity(desiredMotorVelocityRPS)
                            .withFeedForward(DRIVE_MOTOR_MODEL.getVoltage(
                                    additionalFeedforwardMotorTorqueNM, measuredMotorVelocityRadPerSec));
                    case TorqueCurrentFOC -> velocityTorqueCurrentRequest
                            .withVelocity(desiredMotorVelocityRPS)
                            .withFeedForward(DRIVE_MOTOR_MODEL.getCurrent(additionalFeedforwardMotorTorqueNM));
                });
    }

    @Override
    public void requestSteerPositionControl(Rotation2d desiredSteerAbsoluteFacing) {
        steerTalon.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> positionVoltageRequest.withPosition(desiredSteerAbsoluteFacing.getMeasure());
                    case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
                            desiredSteerAbsoluteFacing.getMeasure());
                });
    }

    @Override
    public void setDriveBrake(boolean enableDriveBrake) {
        driveTalon.setNeutralMode(enableDriveBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setSteerBrake(boolean enableSteerBrake) {
        steerTalon.setNeutralMode(enableSteerBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}

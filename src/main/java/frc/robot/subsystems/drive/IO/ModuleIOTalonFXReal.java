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

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.DriveTrainConstants.*;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and CANcoder.
 * Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFXReal extends ModuleIOTalonFX {
    // Inputs from turn motor
    private final StatusSignal<Angle> steerAbsolutePosition;
    private final StatusSignal<AngularVelocity> steerVelocity;
    private final StatusSignal<Voltage> steerMotorAppliedVolts;
    private final StatusSignal<Current> steerMotorStatorCurrent;

    protected final TalonFX steerTalon;
    protected final CANcoder cancoder;

    private final Debouncer steerMotorConnectedDebounce = new Debouncer(0.5);
    private final Debouncer steerEncoderConnectedDebounce = new Debouncer(0.5);

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);

    // Queue to read inputs from odometry thread
    private final Queue<Angle> drivePositionQueue;
    private final Queue<Angle> turnPositionQueue;

    public ModuleIOTalonFXReal(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        super(constants);

        steerTalon = new TalonFX(constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        cancoder = new CANcoder(constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

        // Configure steer motor
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

        // Create turn status signals
        steerAbsolutePosition = cancoder.getAbsolutePosition();
        steerVelocity = steerTalon.getVelocity();
        steerMotorAppliedVolts = steerTalon.getMotorVoltage();
        steerMotorStatorCurrent = steerTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_FREQUENCY, steerAbsolutePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, steerVelocity, steerMotorAppliedVolts, steerMotorStatorCurrent);

        steerTalon.optimizeBusUtilization();

        this.drivePositionQueue = OdometryThread.registerSignalSignal(drivePosition);
        this.turnPositionQueue = OdometryThread.registerSignalSignal(steerAbsolutePosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        var turnStatus = BaseStatusSignal.refreshAll(steerVelocity, steerMotorAppliedVolts, steerMotorStatorCurrent);
        var turnEncoderStatus = BaseStatusSignal.refreshAll(steerAbsolutePosition);

        // Update turn inputs
        inputs.steerMotorConnected = steerMotorConnectedDebounce.calculate(turnStatus.isOK());
        inputs.steerEncoderConnected = steerEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.steerFacing = Rotation2d.fromRotations(steerAbsolutePosition.getValueAsDouble());
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
        inputs.steerMotorAppliedVolts = steerMotorAppliedVolts.getValueAsDouble();
        inputs.steerMotorCurrentAmps = steerMotorStatorCurrent.getValueAsDouble();

        // Update odometry inputs
        inputs.odometryDriveWheelRevolutions = drivePositionQueue.stream()
                .mapToDouble(angle -> angle.in(Rotations) / DRIVE_GEAR_RATIO)
                .toArray();
        inputs.odometrySteerPositions =
                turnPositionQueue.stream().map(Rotation2d::new).toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void requestSteerOpenLoop(Voltage output) {
        steerTalon.setControl(voltageRequest.withOutput(output));
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
    public void setSteerBrake(boolean enableSteerBrake) {
        steerTalon.setNeutralMode(enableSteerBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}

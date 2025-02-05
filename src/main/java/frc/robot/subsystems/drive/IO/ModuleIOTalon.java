// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveTrainConstants.*;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

public class ModuleIOTalon implements ModuleIO {
    private final String name;

    // Module Constants
    private final SwerveModuleConstants moduleConstants;

    // Hardware objects
    private final TalonFX driveTalon;
    private final TalonFX steerTalon;
    private final CANcoder cancoder;

    // Inputs from drive motor
    private final StatusSignal<Angle> driveRotterPosition;
    private final Queue<Angle> driveRotterPositionRotations;
    private final StatusSignal<AngularVelocity> driveRotterVelocity;
    private final StatusSignal<Voltage> driveAppliedVoltage;
    private final StatusSignal<Current> driveMotorCurrentDrawn;

    // Inputs from turn motor
    private final StatusSignal<Angle> steerAbsolutePosition;
    private final Queue<Angle> steerAbsolutePositionCache;
    private final StatusSignal<AngularVelocity> steerFinalMechanismVelocity;
    private final StatusSignal<Voltage> steerAppliedVoltage;
    private final StatusSignal<Current> steerMotorCurrentDrawn;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer steerConnectedDebounce = new Debouncer(0.5);
    private final Debouncer steerEncoderConnectedDebounce = new Debouncer(0.5);

    public ModuleIOTalon(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstants,
            String name) {
        this.name = name;
        this.moduleConstants = moduleConstants;
        driveTalon = new TalonFX(moduleConstants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        steerTalon = new TalonFX(moduleConstants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        cancoder = new CANcoder(moduleConstants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

        // Configure drive motor
        var driveConfig = moduleConstants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = moduleConstants.DriveMotorGains;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = moduleConstants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -moduleConstants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = moduleConstants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_OVER_CURRENT_PROTECTION.in(Amps);
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = DRIVE_OVERHEAT_PROTECTION_TIME.in(Seconds);
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = DRIVE_OVERHEAT_PROTECTION.in(Amps);

        driveConfig.MotorOutput.Inverted = moduleConstants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        var turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0 = moduleConstants.SteerMotorGains;
        turnConfig.Feedback.FeedbackRemoteSensorID = moduleConstants.EncoderId;
        turnConfig.Feedback.FeedbackSensorSource = switch (moduleConstants.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;};
        turnConfig.Feedback.RotorToSensorRatio = moduleConstants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / moduleConstants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * moduleConstants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.Inverted = moduleConstants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> steerTalon.getConfigurator().apply(turnConfig, 0.25));

        // Configure CANCoder
        var cancoderConfig = moduleConstants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = moduleConstants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = moduleConstants.EncoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        // Create drive status signals
        driveRotterPosition = driveTalon.getPosition();
        driveRotterPositionRotations = OdometryThread.registerSignalSignal(driveTalon.getPosition());
        driveRotterVelocity = driveTalon.getVelocity();
        driveAppliedVoltage = driveTalon.getMotorVoltage();
        driveMotorCurrentDrawn = driveTalon.getStatorCurrent();

        // Create turn status signals
        steerAbsolutePosition = cancoder.getAbsolutePosition();
        steerAbsolutePositionCache = OdometryThread.registerSignalSignal(cancoder.getAbsolutePosition());
        steerFinalMechanismVelocity = steerTalon.getVelocity();
        steerAppliedVoltage = steerTalon.getMotorVoltage();
        steerMotorCurrentDrawn = steerTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_FREQUENCY, driveRotterPosition, steerAbsolutePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                driveRotterVelocity,
                driveAppliedVoltage,
                driveMotorCurrentDrawn,
                steerFinalMechanismVelocity,
                steerAppliedVoltage,
                steerMotorCurrentDrawn);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, steerTalon);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(
                driveRotterPosition, driveRotterVelocity, driveAppliedVoltage, driveMotorCurrentDrawn);
        var steerStatus = BaseStatusSignal.refreshAll(
                steerAbsolutePosition, steerFinalMechanismVelocity, steerAppliedVoltage, steerMotorCurrentDrawn);
        var steerEncoderStatus = BaseStatusSignal.refreshAll(steerAbsolutePosition);
        inputs.driveMotorConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.steerMotorConnected = steerConnectedDebounce.calculate(steerStatus.isOK());
        inputs.steerEncoderConnected = steerEncoderConnectedDebounce.calculate(steerEncoderStatus.isOK());

        // Fetch high-frequency drive encoder inputs
        inputs.odometryDriveWheelRevolutions = driveRotterPositionRotations.stream()
                .mapToDouble(value -> value.in(Rotations) / DRIVE_GEAR_RATIO)
                .toArray();
        driveRotterPositionRotations.clear();

        // Fetch high-frequency drive encoder inputs
        inputs.odometrySteerPositions =
                steerAbsolutePositionCache.stream().map(Rotation2d::new).toArray(Rotation2d[]::new);
        steerAbsolutePositionCache.clear();

        // Fetch low frequency position and velocity signals
        inputs.driveWheelFinalRevolutions = driveRotterPosition.getValue().in(Revolutions) / DRIVE_GEAR_RATIO;
        inputs.driveWheelFinalVelocityRevolutionsPerSec =
                driveRotterVelocity.getValue().in(RotationsPerSecond) / DRIVE_GEAR_RATIO;
        inputs.steerFacing = new Rotation2d(steerAbsolutePosition.getValue());
        inputs.steerVelocityRadPerSec = steerFinalMechanismVelocity.getValue().in(RadiansPerSecond);

        // Fetch applied voltage and current
        inputs.driveMotorAppliedVolts = driveAppliedVoltage.getValue().in(Volts);
        inputs.driveMotorCurrentAmps = driveMotorCurrentDrawn.getValue().in(Amps);
        inputs.steerMotorAppliedVolts = steerAppliedVoltage.getValue().in(Volts);
        inputs.steerMotorCurrentAmps = steerMotorCurrentDrawn.getValue().in(Amps);
    }

    boolean driveBrakeEnabled = true;

    @Override
    public void setDriveBrake(boolean enableDriveBrake) {
        if (driveBrakeEnabled == enableDriveBrake) return;
        driveTalon.setNeutralMode(enableDriveBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        this.driveBrakeEnabled = enableDriveBrake;
    }

    boolean steerBrakeEnabled = true;

    @Override
    public void setSteerBrake(boolean enableSteerBrake) {
        if (this.steerBrakeEnabled == enableSteerBrake) return;
        steerTalon.setNeutralMode(enableSteerBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        this.steerBrakeEnabled = enableSteerBrake;
    }

    @Override
    public void requestDriveOpenLoop(Voltage output) {
        driveTalon.setControl(new VoltageOut(output));
    }

    @Override
    public void requestDriveOpenLoop(Current output) {
        driveTalon.setControl(new TorqueCurrentFOC(output));
    }

    @Override
    public void requestSteerOpenLoop(Voltage output) {
        steerTalon.setControl(new VoltageOut(output));
    }

    @Override
    public void requestSteerOpenLoop(Current output) {
        steerTalon.setControl(new TorqueCurrentFOC(output));
    }

    @Override
    public void requestDriveVelocityControl(
            double desiredWheelVelocityRadPerSec, Voltage additionalFeedforwardVoltage) {
        double motorVelocityRotPerSec =
                Units.radiansToRotations(desiredWheelVelocityRadPerSec) * moduleConstants.DriveMotorGearRatio;
        driveTalon.setControl(
                switch (moduleConstants.DriveMotorClosedLoopOutput) {
                    case Voltage -> new VelocityVoltage(motorVelocityRotPerSec)
                            .withFeedForward(additionalFeedforwardVoltage);
                    case TorqueCurrentFOC -> new VelocityTorqueCurrentFOC(motorVelocityRotPerSec);
                });
    }

    @Override
    public void requestSteerPositionControl(Rotation2d desiredSteerAbsoluteFacing) {
        double steerPosition = desiredSteerAbsoluteFacing.getRotations();
        steerTalon.setControl(
                switch (moduleConstants.SteerMotorClosedLoopOutput) {
                    case Voltage -> new PositionVoltage(steerPosition);
                    case TorqueCurrentFOC -> new PositionTorqueCurrentFOC(steerPosition);
                });
    }
}

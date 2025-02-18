// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import static frc.robot.constants.DriveTrainConstants.*;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
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
import frc.robot.subsystems.drive.SwerveModule;

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
    private final OdometryThread.OdometryInput driveRotterPositionRotations;
    private final StatusSignal<AngularVelocity> driveRotterVelocity;
    private final StatusSignal<Voltage> driveAppliedVoltage;
    private final StatusSignal<Current> driveMotorCurrentDrawn;

    // Inputs from turn motor
    private final StatusSignal<Angle> steerAbsolutePosition;
    private final OdometryThread.OdometryInput steerAbsolutePositionCache;
    private final StatusSignal<AngularVelocity> steerFinalMechanismVelocity;
    private final StatusSignal<Voltage> steerAppliedVoltage;
    private final StatusSignal<Current> steerMotorCurrentDrawn;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer steerConnectedDebounce = new Debouncer(0.5);
    private final Debouncer steerEncoderConnectedDebounce = new Debouncer(0.5);

    private final boolean driveConfigurationOK, steerConfigurationOK, canCoderConfigurationOK;

    private static final double timeOutSeconds = 0.2;

    public ModuleIOTalon(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstants,
            String name) {
        this.name = name;
        this.moduleConstants = moduleConstants;
        driveTalon = new TalonFX(moduleConstants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        steerTalon = new TalonFX(moduleConstants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        cancoder = new CANcoder(moduleConstants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

        // Configure drive motor
        var driveCurrentLimit = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(DRIVE_OVER_CURRENT_PROTECTION)
                .withSupplyCurrentLowerTime(DRIVE_OVERHEAT_PROTECTION_TIME)
                .withSupplyCurrentLowerLimit(DRIVE_OVERHEAT_PROTECTION_CURRENT);
        var driveGains = moduleConstants.DriveMotorGains;
        var driveOutput = new MotorOutputConfigs()
                .withInverted(
                        moduleConstants.DriveMotorInverted
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        driveConfigurationOK =
                tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveCurrentLimit, timeOutSeconds))
                        && tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveGains, timeOutSeconds))
                        && tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveOutput, timeOutSeconds));

        // Configure turn motor
        var steerOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                        moduleConstants.SteerMotorInverted
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);
        var steerGains = moduleConstants.SteerMotorGains;
        var steerFeedBack = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(moduleConstants.EncoderId)
                .withRotorToSensorRatio(moduleConstants.SteerMotorGearRatio)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
        var steerCloseLoop = new ClosedLoopGeneralConfigs();
        steerCloseLoop.ContinuousWrap = true;
        steerConfigurationOK = tryUntilOk(5, () -> steerTalon.getConfigurator().apply(steerOutput, timeOutSeconds))
                && tryUntilOk(5, () -> steerTalon.getConfigurator().apply(steerGains, timeOutSeconds))
                && tryUntilOk(5, () -> steerTalon.getConfigurator().apply(steerFeedBack, timeOutSeconds))
                && tryUntilOk(5, () -> steerTalon.getConfigurator().apply(steerCloseLoop, timeOutSeconds));

        // Configure CANCoder
        var cancoderConfig = new MagnetSensorConfigs()
                .withMagnetOffset(moduleConstants.EncoderOffset)
                .withSensorDirection(
                        moduleConstants.EncoderInverted
                                ? SensorDirectionValue.Clockwise_Positive
                                : SensorDirectionValue.CounterClockwise_Positive);
        canCoderConfigurationOK = tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, timeOutSeconds));

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
        inputs.driveMotorConfigurationFailed = !driveConfigurationOK;
        inputs.steerMotorConfigurationFailed = !steerConfigurationOK;
        inputs.steerEncoderConfigurationFailed = !canCoderConfigurationOK;

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
        driveRotterPositionRotations.writeToDoubleInput(
                inputs.odometryDriveWheelRevolutions, rotterPosition -> rotterPosition / DRIVE_GEAR_RATIO);

        // Fetch high-frequency drive encoder inputs
        steerAbsolutePositionCache.writeToInput(inputs.odometrySteerPositions, Rotation2d::fromRotations);

        // Fetch low frequency position and velocity signals
        inputs.driveWheelFinalRevolutions = driveRotterPosition.getValueAsDouble() / DRIVE_GEAR_RATIO;
        inputs.driveWheelFinalVelocityRevolutionsPerSec = driveRotterVelocity.getValueAsDouble() / DRIVE_GEAR_RATIO;
        inputs.steerFacing = Rotation2d.fromRotations(steerAbsolutePosition.getValueAsDouble());
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerFinalMechanismVelocity.getValueAsDouble());

        // Fetch applied voltage and current
        inputs.driveMotorAppliedVolts = driveAppliedVoltage.getValueAsDouble();
        inputs.driveMotorCurrentAmps = driveMotorCurrentDrawn.getValueAsDouble();
        inputs.steerMotorAppliedVolts = steerAppliedVoltage.getValueAsDouble();
        inputs.steerMotorCurrentAmps = steerMotorCurrentDrawn.getValueAsDouble();
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
    public void requestDriveVelocityControl(AngularVelocity desiredMotorVelocity, Torque feedforwardMotorTorque) {
        driveTalon.setControl(
                switch (moduleConstants.DriveMotorClosedLoopOutput) {
                    case Voltage -> new VelocityVoltage(desiredMotorVelocity)
                            .withFeedForward(SwerveModule.calculateFeedforwardVoltage(feedforwardMotorTorque));
                    case TorqueCurrentFOC -> new VelocityTorqueCurrentFOC(desiredMotorVelocity)
                            .withFeedForward(SwerveModule.calculateFeedforwardCurrent(feedforwardMotorTorque));
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

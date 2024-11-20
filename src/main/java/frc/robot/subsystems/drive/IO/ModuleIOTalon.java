// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveTrainConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;

public class ModuleIOTalon implements ModuleIO {
    private final String name;
    private final TalonFX driveTalon;
    private final TalonFX steerTalon;
    private final CANcoder cancoder;

    private final Queue<Angle> driveEncoderUngearedPosition;
    private final StatusSignal<AngularVelocity> driveEncoderUngearedVelocity;
    private final StatusSignal<Voltage> driveMotorAppliedVoltage;
    private final StatusSignal<Current> driveMotorCurrent;

    private final Queue<Angle> steerEncoderAbsolutePosition;
    private final StatusSignal<AngularVelocity> steerEncoderVelocity;
    private final StatusSignal<Voltage> steerMotorAppliedVolts;
    private final StatusSignal<Current> steerMotorCurrent;

    private final BaseStatusSignal[] periodicallyRefreshedSignals;

    private final double DRIVE_GEAR_RATIO;

    public ModuleIOTalon(
            SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants moduleConstants, String name) {
        this.name = name;
        driveTalon = new TalonFX(moduleConstants.DriveMotorId, drivetrainConstants.CANBusName);
        steerTalon = new TalonFX(moduleConstants.SteerMotorId, drivetrainConstants.CANBusName);
        cancoder = new CANcoder(moduleConstants.CANcoderId, drivetrainConstants.CANBusName);

        var driveConfig = moduleConstants.DriveMotorInitialConfigs;
        driveConfig.CurrentLimits.StatorCurrentLimit = DRIVE_CURRENT_LIMIT.in(Amps);
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveTalon.getConfigurator().apply(driveConfig);
        setDriveBrake(true);

        var steerConfig = moduleConstants.SteerMotorInitialConfigs;
        steerConfig.CurrentLimits.StatorCurrentLimit = STEER_CURRENT_LIMIT.in(Amps);
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerTalon.getConfigurator().apply(steerConfig);
        setSteerBrake(true);

        var encoderConfig = moduleConstants.CANcoderInitialConfigs;
        encoderConfig.MagnetSensor.MagnetOffset = moduleConstants.CANcoderOffset;
        cancoder.getConfigurator().apply(encoderConfig);

        driveEncoderUngearedPosition = OdometryThread.registerSignalInput(driveTalon.getPosition());
        driveEncoderUngearedVelocity = driveTalon.getVelocity();
        driveMotorAppliedVoltage = driveTalon.getMotorVoltage();
        driveMotorCurrent = driveTalon.getStatorCurrent();

        steerEncoderAbsolutePosition = OdometryThread.registerSignalInput(cancoder.getAbsolutePosition());
        steerEncoderVelocity = cancoder.getVelocity();
        steerMotorAppliedVolts = steerTalon.getMotorVoltage();
        steerMotorCurrent = steerTalon.getStatorCurrent();

        periodicallyRefreshedSignals = new BaseStatusSignal[] {
            driveEncoderUngearedVelocity, driveMotorAppliedVoltage,
            driveMotorCurrent, steerEncoderVelocity,
            steerMotorAppliedVolts, steerMotorCurrent
        };

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, periodicallyRefreshedSignals);
        driveTalon.optimizeBusUtilization();
        steerTalon.optimizeBusUtilization();

        this.DRIVE_GEAR_RATIO = moduleConstants.DriveMotorGearRatio;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.hardwareConnected =
                BaseStatusSignal.refreshAll(periodicallyRefreshedSignals).isOK();

        inputs.odometryDriveWheelRevolutions = driveEncoderUngearedPosition.stream()
                .mapToDouble(value -> value.in(Rotations) / DRIVE_GEAR_RATIO)
                .toArray();
        driveEncoderUngearedPosition.clear();
        if (inputs.odometryDriveWheelRevolutions.length > 0)
            inputs.driveWheelFinalRevolutions =
                    inputs.odometryDriveWheelRevolutions[inputs.odometryDriveWheelRevolutions.length - 1];

        inputs.odometrySteerPositions =
                steerEncoderAbsolutePosition.stream().map(Rotation2d::new).toArray(Rotation2d[]::new);
        steerEncoderAbsolutePosition.clear();
        if (inputs.odometrySteerPositions.length > 0)
            inputs.steerFacing = inputs.odometrySteerPositions[inputs.odometrySteerPositions.length - 1];

        inputs.driveWheelFinalVelocityRevolutionsPerSec =
                driveEncoderUngearedVelocity.getValue().in(RotationsPerSecond) / DRIVE_GEAR_RATIO;
        inputs.driveMotorAppliedVolts = driveMotorAppliedVoltage.getValue().in(Volts);
        inputs.driveMotorCurrentAmps = driveMotorCurrent.getValue().in(Amps);

        inputs.steerVelocityRadPerSec = steerEncoderVelocity.getValue().in(RadiansPerSecond);
        inputs.steerMotorAppliedVolts = steerMotorAppliedVolts.getValue().in(Volts);
        inputs.steerMotorCurrentAmps = steerMotorCurrent.getValue().in(Amps);
    }

    @Override
    public void setDriveVoltage(double volts) {
        final VoltageOut voltageOut = new VoltageOut(volts).withEnableFOC(false);
        driveTalon.setControl(voltageOut);
    }

    @Override
    public void setSteerPowerPercent(double powerPercent) {
        steerTalon.setControl(new DutyCycleOut(powerPercent).withEnableFOC(true));
    }

    @Override
    public void setDriveBrake(boolean enable) {
        driveTalon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setSteerBrake(boolean enable) {
        steerTalon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}

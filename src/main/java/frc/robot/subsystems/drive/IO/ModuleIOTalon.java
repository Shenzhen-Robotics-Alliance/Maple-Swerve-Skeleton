// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.Queue;

import static frc.robot.constants.DriveTrainConstants.*;

public class ModuleIOTalon implements ModuleIO {
    private final String name;
    private final TalonFX driveTalon;
    private final TalonFX steerTalon;
    private final CANcoder cancoder;

    private final Queue<Double> driveEncoderUngearedRevolutions;
    private final StatusSignal<Double> driveEncoderUngearedRevolutionsPerSecond, driveMotorAppliedVoltage, driveMotorCurrent;

    private final Queue<Double> steerEncoderAbsolutePositionRevolutions;
    private final StatusSignal<Double> steerEncoderVelocityRevolutionsPerSecond, steerMotorAppliedVolts, steerMotorCurrent;

    private final BaseStatusSignal[] periodicallyRefreshedSignals;

    private final double DRIVE_GEAR_RATIO;

    public ModuleIOTalon(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants moduleConstants, String name) {
        this.name = name;
        driveTalon = new TalonFX(moduleConstants.DriveMotorId, drivetrainConstants.CANbusName);
        steerTalon = new TalonFX(moduleConstants.SteerMotorId, drivetrainConstants.CANbusName);
        cancoder = new CANcoder(moduleConstants.CANcoderId, drivetrainConstants.CANbusName);

        var driveConfig = moduleConstants.DriveMotorInitialConfigs;
        driveConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveTalon.getConfigurator().apply(driveConfig);
        driveTalon.setInverted(moduleConstants.DriveMotorInverted);
        setDriveBrake(true);

        var steerConfig = moduleConstants.SteerMotorInitialConfigs;
        steerConfig.CurrentLimits.SupplyCurrentLimit = STEER_CURRENT_LIMIT;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerTalon.getConfigurator().apply(steerConfig);
        steerTalon.setInverted(moduleConstants.SteerMotorInverted);
        setSteerBrake(true);

        var encoderConfig = moduleConstants.CANcoderInitialConfigs;
        encoderConfig.MagnetSensor.MagnetOffset = moduleConstants.CANcoderOffset;
        cancoder.getConfigurator().apply(encoderConfig);

        driveEncoderUngearedRevolutions = OdometryThread.registerSignalInput(driveTalon.getPosition());
        driveEncoderUngearedRevolutionsPerSecond = driveTalon.getVelocity();
        driveMotorAppliedVoltage = driveTalon.getMotorVoltage();
        driveMotorCurrent = driveTalon.getSupplyCurrent();

        steerEncoderAbsolutePositionRevolutions = OdometryThread.registerSignalInput(cancoder.getAbsolutePosition());
        steerEncoderVelocityRevolutionsPerSecond = cancoder.getVelocity();
        steerMotorAppliedVolts = steerTalon.getMotorVoltage();
        steerMotorCurrent = steerTalon.getSupplyCurrent();

        periodicallyRefreshedSignals = new BaseStatusSignal[]{
                driveEncoderUngearedRevolutionsPerSecond,
                driveMotorAppliedVoltage, driveMotorCurrent,
                steerEncoderVelocityRevolutionsPerSecond,
                steerMotorAppliedVolts, steerMotorCurrent
        };

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, periodicallyRefreshedSignals);
        driveTalon.optimizeBusUtilization();
        steerTalon.optimizeBusUtilization();

        this.DRIVE_GEAR_RATIO = moduleConstants.DriveMotorGearRatio;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.hardwareConnected = BaseStatusSignal.refreshAll(periodicallyRefreshedSignals).isOK();

        inputs.odometryDriveWheelRevolutions = driveEncoderUngearedRevolutions.stream()
                .mapToDouble(value -> value / DRIVE_GEAR_RATIO)
                .toArray();
        driveEncoderUngearedRevolutions.clear();
        if (inputs.odometryDriveWheelRevolutions.length > 0)
            inputs.driveWheelFinalRevolutions = inputs.odometryDriveWheelRevolutions[inputs.odometryDriveWheelRevolutions.length-1];

        inputs.odometrySteerPositions = steerEncoderAbsolutePositionRevolutions.stream()
                .map(this::getSteerFacingFromCANCoderReading)
                .toArray(Rotation2d[]::new);
        steerEncoderAbsolutePositionRevolutions.clear();
        if (inputs.odometrySteerPositions.length > 0)
            inputs.steerFacing = inputs.odometrySteerPositions[inputs.odometrySteerPositions.length-1];

        inputs.driveWheelFinalVelocityRevolutionsPerSec = driveEncoderUngearedRevolutionsPerSecond.getValueAsDouble() / DRIVE_GEAR_RATIO;
        inputs.driveMotorAppliedVolts = driveMotorAppliedVoltage.getValueAsDouble();
        inputs.driveMotorCurrentAmps = driveMotorCurrent.getValueAsDouble();

        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerEncoderVelocityRevolutionsPerSecond.getValueAsDouble());
        inputs.steerMotorAppliedVolts = steerMotorAppliedVolts.getValueAsDouble();
        inputs.steerMotorCurrentAmps = steerMotorCurrent.getValueAsDouble();
    }

    private Rotation2d getSteerFacingFromCANCoderReading(double canCoderReadingRotations) {
        return Rotation2d.fromRotations(canCoderReadingRotations);
    }

    @Override
    public void setDriveVoltage(double volts) {
        final VoltageOut voltageOut = new VoltageOut(volts).withEnableFOC(false);
        driveTalon.setControl(voltageOut);
    }

    @Override
    public void setSteerPowerPercent(double powerPercent) {
       steerTalon.setControl(new DutyCycleOut(powerPercent)
               .withEnableFOC(true));
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

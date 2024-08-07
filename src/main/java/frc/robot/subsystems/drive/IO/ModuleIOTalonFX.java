// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utils.Config.MapleConfigFile;

import java.util.Queue;

public class ModuleIOTalonFX implements ModuleIO {
    private final String name;
    private final TalonFX driveTalon;
    private final TalonFX steerTalon;
    private final CANcoder cancoder;

    private final Queue<Double> driveEncoderUngearedRevolutions;
    private final StatusSignal<Double> driveEncoderUngearedRevolutionsPerSecond, driveMotorAppliedVoltage, driveMotorCurrent;

    private final Queue<Double> steerEncoderAbsolutePositionRevolutions;
    private final StatusSignal<Double> steerEncoderVelocityRevolutionsPerSecond, steerMotorAppliedVolts, steerMotorCurrent;

    private final BaseStatusSignal[] periodicallyRefreshedSignals;

    private final Rotation2d absoluteEncoderOffset;
    private final double DRIVE_GEAR_RATIO;

    public ModuleIOTalonFX(MapleConfigFile.ConfigBlock moduleConfigs, MapleConfigFile.ConfigBlock generalConfigs) {
        this.name = moduleConfigs.getBlockName();
        driveTalon = new TalonFX(moduleConfigs.getIntConfig("drivingMotorID"), Constants.SwerveDriveChassisConfigs.CHASSIS_CANBUS);
        steerTalon = new TalonFX(moduleConfigs.getIntConfig("steeringMotorID"), Constants.SwerveDriveChassisConfigs.CHASSIS_CANBUS);
        cancoder = new CANcoder(moduleConfigs.getIntConfig("steeringEncoderID"), Constants.SwerveDriveChassisConfigs.CHASSIS_CANBUS);
        absoluteEncoderOffset = new Rotation2d(moduleConfigs.getDoubleConfig("steeringEncoderReadingAtOrigin")); // MUST BE CALIBRATED

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveModuleConfigs.DRIVING_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveTalon.getConfigurator().apply(driveConfig);
        setDriveBrake(true);

        var steerConfig = new TalonFXConfiguration();
        steerConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveModuleConfigs.STEERING_CURRENT_LIMIT;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerTalon.getConfigurator().apply(steerConfig);
        steerTalon.setInverted(moduleConfigs.getIntConfig("steeringMotorInverted") != 0);
        setSteerBrake(true);

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

        this.DRIVE_GEAR_RATIO = generalConfigs.getDoubleConfig("overallGearRatio");
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
        return Rotation2d.fromRotations(canCoderReadingRotations).minus(absoluteEncoderOffset);
    }

    @Override
    public void setDriveSpeedPercent(double speedPercent) {
        driveTalon.setControl(new DutyCycleOut(speedPercent)
                .withEnableFOC(false));
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

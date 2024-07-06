// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

import java.util.Arrays;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveTalon;
    private final TalonFX steerTalon;
    private final CANcoder cancoder;

    private final OdometryThreadReal.OdometryDoubleInput driveEncoderUngearedRevolutions;
    private final StatusSignal<Double> driveEncoderUngearedRevolutionsPerSecond;
    private final StatusSignal<Double> driveMotorAppliedVoltage;
    private final StatusSignal<Double> driveMotorCurrent;

    private final OdometryThreadReal.OdometryDoubleInput steerEncoderAbsolutePositionRevolutions;
    private final StatusSignal<Double> steerEncoderVelocityRevolutionsPerSecond;
    private final StatusSignal<Double> steerMotorAppliedVolts;
    private final StatusSignal<Double> steerMotorCurrent;

    // Gear ratios for SDS MK4i L2, adjust as necessary
    private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOTalonFX(int index) {
        switch (index) {
            case 0:
                driveTalon = new TalonFX(3, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                steerTalon = new TalonFX(4, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                cancoder = new CANcoder(10, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                absoluteEncoderOffset = new Rotation2d(3.3195344249845276); // MUST BE CALIBRATED
                break;
            case 1:
                driveTalon = new TalonFX(6, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                steerTalon = new TalonFX(5, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                cancoder = new CANcoder(11, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                absoluteEncoderOffset = new Rotation2d(1.7564080021290591); // MUST BE CALIBRATED
                break;
            case 2:
                driveTalon = new TalonFX(1, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                steerTalon = new TalonFX(2, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                cancoder = new CANcoder(9, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                absoluteEncoderOffset = new Rotation2d(0.34974761963792617); // MUST BE CALIBRATED
                break;
            case 3:
                driveTalon = new TalonFX(8, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                steerTalon = new TalonFX(7, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                cancoder = new CANcoder(12, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
                absoluteEncoderOffset = new Rotation2d(0.10737865515199488); // MUST BE CALIBRATED
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveTalon.getConfigurator().apply(driveConfig);
        setDriveBrakeMode(true);

        var turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerTalon.getConfigurator().apply(turnConfig);
        setTurnBrakeMode(true);

        driveEncoderUngearedRevolutions = OdometryThread.registerSignalInput(driveTalon.getPosition());
        driveEncoderUngearedRevolutionsPerSecond = driveTalon.getVelocity();
        driveMotorAppliedVoltage = driveTalon.getMotorVoltage();
        driveMotorCurrent = driveTalon.getSupplyCurrent();

        steerEncoderAbsolutePositionRevolutions = OdometryThread.registerSignalInput(cancoder.getAbsolutePosition());
        steerEncoderVelocityRevolutionsPerSecond = cancoder.getVelocity();
        steerMotorAppliedVolts = steerTalon.getMotorVoltage();
        steerMotorCurrent = steerTalon.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveEncoderUngearedRevolutionsPerSecond,
                driveMotorAppliedVoltage,
                driveMotorCurrent,
                steerEncoderVelocityRevolutionsPerSecond,
                steerMotorAppliedVolts,
                steerMotorCurrent);
        driveTalon.optimizeBusUtilization();
        steerTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                driveEncoderUngearedRevolutionsPerSecond,
                driveMotorAppliedVoltage,
                driveMotorCurrent,
                steerEncoderVelocityRevolutionsPerSecond,
                steerMotorAppliedVolts,
                steerMotorCurrent);

        inputs.driveWheelFinalRevolutions = driveEncoderUngearedRevolutions.getLatest() / DRIVE_GEAR_RATIO;
        inputs.driveWheelFinalVelocityRevolutionsPerSec = Units.rotationsToRadians(driveEncoderUngearedRevolutionsPerSecond.getValueAsDouble()) / DRIVE_GEAR_RATIO;
        inputs.driveMotorAppliedVolts = driveMotorAppliedVoltage.getValueAsDouble();
        inputs.driveMotorCurrentAmps = driveMotorCurrent.getValueAsDouble();

        inputs.steerFacing = getSteerFacingFromCANCoderReading(steerEncoderAbsolutePositionRevolutions.getLatest());
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerEncoderVelocityRevolutionsPerSecond.getValueAsDouble());
        inputs.steerMotorAppliedVolts = steerMotorAppliedVolts.getValueAsDouble();
        inputs.steerMotorCurrentAmps = steerMotorCurrent.getValueAsDouble();

        inputs.odometryDriveWheelRevolutions = Arrays.stream(driveEncoderUngearedRevolutions.getValuesSincePreviousPeriod())
                .mapToDouble((Double value) -> value / DRIVE_GEAR_RATIO)
                .toArray();
        inputs.odometrySteerPositions = Arrays.stream(steerEncoderAbsolutePositionRevolutions.getValuesSincePreviousPeriod())
                .map(this::getSteerFacingFromCANCoderReading)
                .toArray(Rotation2d[]::new);
    }

    private Rotation2d getSteerFacingFromCANCoderReading(double canCoderReadingRotations) {
        return Rotation2d.fromRotations(canCoderReadingRotations).minus(absoluteEncoderOffset);
    }

    @Override
    public void setDrivePower(double power) {
        driveTalon.set(power);
    }

    @Override
    public void setSteerPower(double power) {
       steerTalon.set(power);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveTalon.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = isTurnMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        steerTalon.getConfigurator().apply(config);
    }
}

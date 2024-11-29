// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.constants.DriveTrainConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO or NEO 550), and
 * analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware configurations
 * (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward motion on the drive motor
 * will propel the robot forward) and copy the reported values from the absolute encoders using AdvantageScope. These
 * values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSpark implements ModuleIO {
    // Gear ratios for SDS MK4i L2, adjust as necessary
    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    private final SparkFlex driveSparkFlex;
    private final SparkMax turnSparkMax;
    private final CANcoder cancoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerRelativeEncoder;
    private final Queue<Double> drivePositionInput;
    private final Queue<Double> steerRelativeEncoderPositionUngeared;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOSpark(int index) {
        switch (index) {
            case 0 -> {
                driveSparkFlex = new SparkFlex(41, SparkLowLevel.MotorType.kBrushless);
                turnSparkMax = new SparkMax(51, SparkLowLevel.MotorType.kBrushless);
                cancoder = new CANcoder(31);
                absoluteEncoderOffset = Rotation2d.fromRotations(0.290527);
            }
            case 1 -> {
                driveSparkFlex = new SparkFlex(42, SparkLowLevel.MotorType.kBrushless);
                turnSparkMax = new SparkMax(52, SparkLowLevel.MotorType.kBrushless);
                cancoder = new CANcoder(32);
                absoluteEncoderOffset = Rotation2d.fromRotations(0.095215);
            }
            case 2 -> {
                driveSparkFlex = new SparkFlex(43, SparkLowLevel.MotorType.kBrushless);
                turnSparkMax = new SparkMax(53, SparkLowLevel.MotorType.kBrushless);
                cancoder = new CANcoder(33);
                absoluteEncoderOffset = Rotation2d.fromRotations(-0.360352);
            }
            case 3 -> {
                driveSparkFlex = new SparkFlex(44, SparkLowLevel.MotorType.kBrushless);
                turnSparkMax = new SparkMax(54, SparkLowLevel.MotorType.kBrushless);
                cancoder = new CANcoder(34);
                absoluteEncoderOffset = Rotation2d.fromRotations(-0.5 + 0.41254);
            }
            default -> throw new RuntimeException("Invalid module index");
        }

        driveEncoder = driveSparkFlex.getEncoder();
        steerRelativeEncoder = turnSparkMax.getEncoder();

        SparkFlexConfig driveConfig = new SparkFlexConfig();
        SparkMaxConfig turnConfig = new SparkMaxConfig();

        driveConfig.smartCurrentLimit(40).voltageCompensation(12.0).idleMode(SparkBaseConfig.IdleMode.kBrake);
        driveConfig.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(2);
        driveConfig.signals.primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY));

        turnConfig
                .inverted(isTurnMotorInverted)
                .smartCurrentLimit(30)
                .voltageCompensation(12.0)
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
        turnConfig.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(2);
        driveConfig.signals.primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY));

        driveSparkFlex.configure(
                driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        turnSparkMax.configure(
                turnConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        this.drivePositionInput = OdometryThread.registerInput(driveEncoder::getPosition);
        this.steerRelativeEncoderPositionUngeared = OdometryThread.registerInput(steerRelativeEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveWheelFinalRevolutions = driveEncoder.getPosition() / DRIVE_GEAR_RATIO;
        final double RPM_TO_REVOLUTIONS_PER_SECOND = 1.0 / 60.0;
        inputs.driveWheelFinalVelocityRevolutionsPerSec =
                driveEncoder.getVelocity() / DRIVE_GEAR_RATIO * RPM_TO_REVOLUTIONS_PER_SECOND;

        inputs.driveMotorAppliedVolts = driveSparkFlex.getAppliedOutput() * driveSparkFlex.getBusVoltage();
        inputs.driveMotorCurrentAmps = driveSparkFlex.getOutputCurrent();

        inputs.steerFacing = Rotation2d.fromRotations(steerRelativeEncoder.getPosition() / STEER_GEAR_RATIO)
                .minus(steerRelativePositionEncoderOffset);
        inputs.steerVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(steerRelativeEncoder.getVelocity() / STEER_GEAR_RATIO);

        inputs.steerMotorAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.steerMotorCurrentAmps = turnSparkMax.getOutputCurrent();

        inputs.odometryDriveWheelRevolutions = drivePositionInput.stream()
                .mapToDouble(value -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
                .toArray();
        drivePositionInput.clear();
        inputs.odometrySteerPositions = steerRelativeEncoderPositionUngeared.stream()
                .map(value ->
                        Rotation2d.fromRotations(value / STEER_GEAR_RATIO).minus(steerRelativePositionEncoderOffset))
                .toArray(Rotation2d[]::new);
        steerRelativeEncoderPositionUngeared.clear();
    }

    private Rotation2d steerRelativePositionEncoderOffset = new Rotation2d();

    @Override
    public void calibrate() {
        final Rotation2d steerActualFacing = Rotation2d.fromDegrees(
                        cancoder.getPosition().getValue().in(Degrees))
                .minus(absoluteEncoderOffset);
        final Rotation2d relativeEncoderReportedFacing =
                Rotation2d.fromRotations(steerRelativeEncoder.getPosition() / STEER_GEAR_RATIO);
        /* reported - offset = actual, so offset = reported - actual */
        steerRelativePositionEncoderOffset = relativeEncoderReportedFacing.minus(steerActualFacing);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSparkFlex.set(volts / RobotController.getBatteryVoltage());
    }

    @Override
    public void setSteerPowerPercent(double powerPercent) {
        turnSparkMax.set(powerPercent);
    }
}

// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import java.util.Queue;

import static frc.robot.constants.DriveTrainConstants.*;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSpark implements ModuleIO {
    // Gear ratios for SDS MK4i L2, adjust as necessary
    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    private final CANSparkMax driveSparkMax;
    private final CANSparkMax steerSparkMax;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerRelativeEncoder;
    private final AnalogInput turnAbsoluteEncoder;
    private final Queue<Double> drivePositionInput;
    private final Queue<Double> steerRelativeEncoderPositionUngeared;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOSpark(int index) {
        switch (index) {
            case 0 -> {
                driveSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(2, MotorType.kBrushless);
                turnAbsoluteEncoder = new AnalogInput(0);
                absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
            }
            case 1 -> {
                driveSparkMax = new CANSparkMax(3, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(4, MotorType.kBrushless);
                turnAbsoluteEncoder = new AnalogInput(1);
                absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
            }
            case 2 -> {
                driveSparkMax = new CANSparkMax(5, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(6, MotorType.kBrushless);
                turnAbsoluteEncoder = new AnalogInput(2);
                absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
            }
            case 3 -> {
                driveSparkMax = new CANSparkMax(7, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(8, MotorType.kBrushless);
                turnAbsoluteEncoder = new AnalogInput(3);
                absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
            }
            default -> throw new RuntimeException("Invalid module index");
        }

        driveSparkMax.restoreFactoryDefaults();
        steerSparkMax.restoreFactoryDefaults();

        driveSparkMax.setCANTimeout(250);
        steerSparkMax.setCANTimeout(250);

        driveEncoder = driveSparkMax.getEncoder();
        steerRelativeEncoder = steerSparkMax.getEncoder();

        steerSparkMax.setInverted(isTurnMotorInverted);
        driveSparkMax.setSmartCurrentLimit(40);
        steerSparkMax.setSmartCurrentLimit(30);
        driveSparkMax.enableVoltageCompensation(12.0);
        steerSparkMax.enableVoltageCompensation(12.0);

        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(10);
        driveEncoder.setAverageDepth(2);

        steerRelativeEncoder.setPosition(0.0);
        steerRelativeEncoder.setMeasurementPeriod(10);
        steerRelativeEncoder.setAverageDepth(2);

        driveSparkMax.setCANTimeout(0);
        steerSparkMax.setCANTimeout(0);

        driveSparkMax.setPeriodicFramePeriod(
                PeriodicFrame.kStatus2,
                (int) (1000.0 / ODOMETRY_FREQUENCY)
        );
        steerSparkMax.setPeriodicFramePeriod(
                PeriodicFrame.kStatus2,
                (int) (1000.0 / ODOMETRY_FREQUENCY)
        );
        this.drivePositionInput = OdometryThread.registerInput(driveEncoder::getPosition);
        this.steerRelativeEncoderPositionUngeared = OdometryThread.registerInput(steerRelativeEncoder::getPosition);

        driveSparkMax.burnFlash();
        steerSparkMax.burnFlash();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveWheelFinalRevolutions = driveEncoder.getPosition() / DRIVE_GEAR_RATIO;
        final double RPM_TO_REVOLUTIONS_PER_SECOND = 1.0/60.0;
        inputs.driveWheelFinalVelocityRevolutionsPerSec = driveEncoder.getVelocity() / DRIVE_GEAR_RATIO
                * RPM_TO_REVOLUTIONS_PER_SECOND;

        inputs.driveMotorAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveMotorCurrentAmps = driveSparkMax.getOutputCurrent();

        inputs.steerFacing = Rotation2d.fromRotations(steerRelativeEncoder.getPosition() / STEER_GEAR_RATIO).minus(steerRelativePositionEncoderOffset);
        inputs.steerVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                steerRelativeEncoder.getVelocity() / STEER_GEAR_RATIO
        );

        inputs.steerMotorAppliedVolts = steerSparkMax.getAppliedOutput() * steerSparkMax.getBusVoltage();
        inputs.steerMotorCurrentAmps = steerSparkMax.getOutputCurrent();

        inputs.odometryDriveWheelRevolutions = drivePositionInput.stream()
                .mapToDouble(value -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
                .toArray();
        drivePositionInput.clear();
        inputs.odometrySteerPositions = steerRelativeEncoderPositionUngeared.stream()
                .map(value -> Rotation2d.fromRotations(value / STEER_GEAR_RATIO).minus(steerRelativePositionEncoderOffset))
                .toArray(Rotation2d[]::new);
        steerRelativeEncoderPositionUngeared.clear();
    }

    private Rotation2d steerRelativePositionEncoderOffset = new Rotation2d();
    @Override
    public void calibrate() {
        final Rotation2d steerActualFacing = Rotation2d.fromRotations(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V()
                ).minus(absoluteEncoderOffset);
        final Rotation2d relativeEncoderReportedFacing = Rotation2d.fromRotations(steerRelativeEncoder.getPosition() / STEER_GEAR_RATIO);
        /* reported - offset = actual, so offset = reported - actual */
        steerRelativePositionEncoderOffset = relativeEncoderReportedFacing.minus(steerActualFacing);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSparkMax.set(volts / RobotController.getBatteryVoltage());
    }

    @Override
    public void setSteerPowerPercent(double powerPercent) {
        steerSparkMax.set(powerPercent);
    }

    @Override
    public void setDriveBrake(boolean enable) {
        driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setSteerBrake(boolean enable) {
        steerSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}

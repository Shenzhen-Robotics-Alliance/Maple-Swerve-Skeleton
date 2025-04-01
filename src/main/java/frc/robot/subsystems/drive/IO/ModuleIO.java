// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DriveTrainConstants;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public boolean driveMotorConfigurationFailed = false;
        public boolean steerMotorConfigurationFailed = false;
        public boolean steerEncoderConfigurationFailed = false;

        public boolean hardwareCurrentlyConnected = false;
        public double driveWheelFinalRevolutions = 0.0;
        public double driveWheelFinalVelocityRevolutionsPerSec = 0.0;
        public double driveMotorAppliedVolts = 0.0;
        public double driveMotorCurrentAmps = 0;

        public Rotation2d steerFacing = new Rotation2d();
        public double steerVelocityRadPerSec = 0.0;
        public double steerMotorAppliedVolts = 0.0;
        public double steerMotorCurrentAmps = 0.0;

        public double[] odometryDriveWheelRevolutions = new double[DriveTrainConstants.ODOMETRY_CACHE_CAPACITY];
        public Rotation2d[] odometrySteerPositions = new Rotation2d[DriveTrainConstants.ODOMETRY_CACHE_CAPACITY];
    }

    /** Updates the inputs */
    void updateInputs(ModuleIOInputs inputs);

    default void calibrate() {}

    /**
     * Run the drive motor at a specified voltage open-loop control
     *
     * @param outputVolts the desired voltage output, from -12v to 12v
     */
    default void requestDriveOpenLoop(double outputVolts) {}

    /**
     * Run the steer motor at a specified voltage open-loop control
     *
     * @param outputVolts the desired voltage output, from -12v to 12v
     */
    default void requestSteerOpenLoop(double outputVolts) {}

    /**
     * Runs a velocity close-loop control on the drive motor
     *
     * @param desiredMotorVelocityRadPerSec the desired angular velocity of the motor, in radians / second
     * @param feedforwardMotorVoltage additional feed-forward voltage to add to the motor output
     */
    default void requestDriveVelocityControl(double desiredMotorVelocityRadPerSec, double feedforwardMotorVoltage) {}

    /**
     * Runs a position close-loop control on the steer motor
     *
     * @param desiredSteerAbsoluteFacing the desired facing of the steer
     */
    default void requestSteerPositionControl(Rotation2d desiredSteerAbsoluteFacing) {}

    /** Enable or disable brake mode on the drive motor. */
    default void setDriveBrake(boolean enableDriveBrake) {}

    /** Enable or disable brake mode on the turn motor. */
    default void setSteerBrake(boolean enableSteerBrake) {}
}

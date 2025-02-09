// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double driveWheelFinalRevolutions = 0.0;
        public double driveWheelFinalVelocityRevolutionsPerSec = 0.0;
        public double driveMotorAppliedVolts = 0.0;
        public double driveMotorCurrentAmps = 0;

        public Rotation2d steerFacing = new Rotation2d();
        public double steerVelocityRadPerSec = 0.0;
        public double steerMotorAppliedVolts = 0.0;
        public double steerMotorCurrentAmps = 0.0;

        public double[] odometryDriveWheelRevolutions = new double[] {};
        public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};

        public boolean driveMotorConnected = false;
        public boolean steerMotorConnected = false;
        public boolean steerEncoderConnected = false;
    }

    /** Updates the inputs */
    void updateInputs(ModuleIOInputs inputs);

    default void calibrate() {}

    /**
     * Run the drive motor at a specified voltage open-loop control
     *
     * @param output the desired voltage output, from -12v to 12v
     */
    default void requestDriveOpenLoop(Voltage output) {}

    /**
     * Run the drive motor at a specified current open-loop control
     *
     * @param output the desired current output
     */
    default void requestDriveOpenLoop(Current output) {}

    /**
     * Run the steer motor at a specified voltage open-loop control
     *
     * @param output the desired voltage output, from -12v to 12v
     */
    default void requestSteerOpenLoop(Voltage output) {}

    /**
     * Run the steer motor at a specified current open-loop control
     *
     * @param output the desired current output
     */
    default void requestSteerOpenLoop(Current output) {}

    /**
     * Runs a velocity close-loop control on the drive motor
     *
     * @param desiredMotorVelocity the desired angular velocity of the motor, in radians / second
     * @param feedforwardMotorTorque torque feedforward to add to the motor output
     */
    default void requestDriveVelocityControl(AngularVelocity desiredMotorVelocity, Torque feedforwardMotorTorque) {}

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

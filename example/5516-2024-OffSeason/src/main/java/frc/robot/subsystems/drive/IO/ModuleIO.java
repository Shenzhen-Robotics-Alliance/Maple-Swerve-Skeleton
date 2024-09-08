// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

        public double[] odometryDriveWheelRevolutions = new double[]{};
        public Rotation2d[] odometrySteerPositions = new Rotation2d[]{};

        public boolean hardwareConnected = false;
    }

    /**
     * Updates the inputs
     */
    void updateInputs(ModuleIOInputs inputs);

    default void calibrate() {}

    /**
     * Run the drive motor at the specified percent speed.
     * @param speedPercent from -1 to 1, where 1 is the forward direction of the wheel
     */
    default void setDriveVoltage(double speedPercent) {}

    /**
     * Run the turn motor at the specified percent power.
     * @param powerPercent from -1 to 1, where 1 is counter-clockwise
     */
    default void setSteerPowerPercent(double powerPercent) {}

    /**
     * Enable or disable brake mode on the drive motor.
     */
    default void setDriveBrake(boolean enable) {}

    /**
     * Enable or disable brake mode on the turn motor.
     */
    default void setSteerBrake(boolean enable) {}
}

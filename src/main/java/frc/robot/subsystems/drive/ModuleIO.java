// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[]{};

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[]{};

        public double[] odometryTimestamps = new double[]{};
        public double[] odometryDrivePositionsRad = new double[]{};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[]{};
    }

    /**
     * Updates the set of loggable inputs.
     */
    void updateInputs(ModuleIOInputs inputs);

    /**
     * Run the drive motor at the specified voltage.
     */
    default void setDriveVoltage(double volts) {}

    /**
     * Run the turn motor at the specified voltage.
     */
    default void setTurnVoltage(double volts) {}

    /**
     * Enable or disable brake mode on the drive motor.
     */
    default void setDriveBrakeMode(boolean enable) {}

    /**
     * Enable or disable brake mode on the turn motor.
     */
    default void setTurnBrakeMode(boolean enable) {}
}

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface PitchIO {
    @AutoLog
    class PitchInputs {
        double pitchSuppliedCurrentAmps = 0.0;
        double pitchAngleRad = 0.0;
        double pitchAngularVelocityRadPerSec = 0.0;
        boolean calibrated = false;
    }

    void updateInputs(PitchInputs pitchInputs);

    default void runPitchVoltage(double volts) {}

    default void setPitchLock(boolean enabled) {}
}

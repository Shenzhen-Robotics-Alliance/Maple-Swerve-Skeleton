package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        boolean lowerBeamBreakBlocked = true;
        boolean upperBeamBreakerBlocked = true;
        double motorCurrent = 0.0;
    }

    void updateInputs(IntakeInputs inputs);

    default void runIntakeVoltage(double volts) {}
}

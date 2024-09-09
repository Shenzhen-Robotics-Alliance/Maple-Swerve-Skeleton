package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlyWheelIO {
    @AutoLog
    class FlyWheelsInputs {
        double flyWheelPositionRevs = 0.0;
        double flyWheelVelocityRevsPerSec = 0.0;
        double supplyCurrentAmps = 0.0;
    }

    void updateInputs(FlyWheelsInputs inputs);

    default void runVoltage(double volts) {}
}

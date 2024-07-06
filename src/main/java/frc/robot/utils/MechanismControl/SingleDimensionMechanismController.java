package frc.robot.utils.MechanismControl;

public interface SingleDimensionMechanismController {
    double getMotorPower(double mechanismVelocity, double mechanismPosition);
}

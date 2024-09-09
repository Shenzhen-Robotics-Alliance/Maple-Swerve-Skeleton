package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeIOSim;

public class FlyWheelIOSim implements FlyWheelIO {
    private final IntakeIOSim intakeIOSim;
    private final DCMotorSim dcMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1.5, 0.00064);

    public FlyWheelIOSim(IntakeIOSim intakeIOSim) {
        this.intakeIOSim = intakeIOSim;
    }

    @Override
    public void updateInputs(FlyWheelsInputs inputs) {
        dcMotorSim.update(Robot.defaultPeriodSecs);
        inputs.supplyCurrentAmps = dcMotorSim.getCurrentDrawAmps();
        inputs.flyWheelVelocityRevsPerSec = Units.radiansToRotations(
                dcMotorSim.getAngularVelocityRadPerSec()
        );
        inputs.flyWheelPositionRevs = Units.radiansToRotations(
                dcMotorSim.getAngularPositionRad()
        );

        intakeIOSim.setShooterRunning(inputs.flyWheelVelocityRevsPerSec * 60 > 200);
    }

    @Override
    public void runVoltage(double volts) {
        dcMotorSim.setInputVoltage(volts);
    }
}

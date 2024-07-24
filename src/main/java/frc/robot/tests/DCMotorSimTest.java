package frc.robot.tests;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class DCMotorSimTest implements UnitTest {
    private final DCMotorSim dcMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), Constants.ChassisDefaultConfigs.DEFAULT_GEAR_RATIO, 0.08);


    @Override
    public void testStart() {

    }

    private final XboxController xboxController = new XboxController(1);
    @Override
    public void testPeriodic() {
        dcMotorSim.setInputVoltage(-12 * xboxController.getLeftY());
        dcMotorSim.update(0.02);
        Logger.recordOutput("test/DriveWheelSimOutputVelocity", dcMotorSim.getAngularVelocityRPM());
    }
}

package frc.robot.tests;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.CompetitionFieldUtils.Simulation.Crescendo2024FieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.OpponentRobotSimulation;

public class PhysicsSimulationTest implements UnitTest {
    private final OpponentRobotSimulation opponentRobotSimulation = new OpponentRobotSimulation();
    private final Crescendo2024FieldSimulation fieldSimulation = new Crescendo2024FieldSimulation(opponentRobotSimulation);
    @Override
    public void testStart() {
        fieldSimulation.addRobot(opponentRobotSimulation);
    }

    private final XboxController xboxController = new XboxController(1);
    @Override
    public void testPeriodic() {
        opponentRobotSimulation.testDrivingPhysicsWithJoystick(xboxController);
        fieldSimulation.updateSimulationWorld();
    }
}

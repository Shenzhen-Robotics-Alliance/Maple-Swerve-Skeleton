package frc.robot.tests;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.CompetitionFieldUtils.Simulation.Crescendo2024FieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.OpponentRobotSimulation;

public class PhysicsSimulationTest implements UnitTest {
    private final OpponentRobotSimulation opponentRobotSimulation = new OpponentRobotSimulation(0);
    private final Crescendo2024FieldSimulation fieldSimulation = new Crescendo2024FieldSimulation(opponentRobotSimulation);

    public PhysicsSimulationTest() {
        fieldSimulation.addRobot(opponentRobotSimulation);
    }
    @Override
    public void testStart() {
        opponentRobotSimulation.setPose(new Pose2d(15.2, 2.5, Rotation2d.fromRotations(0.5)));
    }

    private final XboxController xboxController = new XboxController(1);
    @Override
    public void testPeriodic() {
        opponentRobotSimulation.testDrivingPhysicsWithJoystick(xboxController);
        fieldSimulation.updateSimulationWorld();
    }
}

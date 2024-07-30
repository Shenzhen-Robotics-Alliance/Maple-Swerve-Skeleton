package frc.robot.tests;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.Simulation.Crescendo2024FieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.OpponentRobotSimulation;

public class PhysicsSimulationTest extends Command {
    private final OpponentRobotSimulation opponentRobotSimulation;
    private final Crescendo2024FieldSimulation fieldSimulation;

    public PhysicsSimulationTest() {
        this.opponentRobotSimulation = new OpponentRobotSimulation(0);
        this.fieldSimulation = new Crescendo2024FieldSimulation(opponentRobotSimulation);
    }

    @Override
    public void initialize() {
        opponentRobotSimulation.setPose(new Pose2d(15.2, 2.5, Rotation2d.fromRotations(0.5)));
        fieldSimulation.resetFieldForAuto();

        System.out.println("opponent robot profile: " + opponentRobotSimulation.profile);
    }

    private final XboxController xboxController = new XboxController(1);

//    @Override
//    public void execute() {
//        opponentRobotSimulation.testDrivingPhysicsWithJoystick(xboxController);
//        fieldSimulation.updateSimulationWorld();
//        if (xboxController.getAButton())
//            fieldSimulation.addGamePiece(new Crescendo2024FieldObjects.NoteOnFieldSimulated(
//                    new Translation2d(3, 3)
//            ));
//    }
}

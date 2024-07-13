package frc.robot.utils.CompetitionFieldUtils.Simulation;

import frc.robot.utils.CompetitionFieldUtils.FieldObjects.RobotOnField;
public class Crescendo2024FieldSimulation extends CompetitionFieldSimulation {
    public Crescendo2024FieldSimulation(RobotOnField robot) {
        super(robot, new CrescendoFieldObstaclesMap());
    }

    // TODO: notes simulation and match simulation

    public static final class CrescendoFieldObstaclesMap extends FieldObstaclesMap {
        public CrescendoFieldObstaclesMap() {
            super();
            // super.addBorderLine(new Translation2d(1,2), new Translation2d(1,5));
            // TODO: crescendo field
        }
    }
}

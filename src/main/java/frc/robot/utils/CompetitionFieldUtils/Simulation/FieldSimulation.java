package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;
import frc.robot.utils.MapleMaths.GeometryConvertor;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;

import java.util.ArrayList;
import java.util.List;

/**
 * the class that simulates the physical behavior of all the objects on field
 * should only be created during a robot simulation (not in real or replay mode)
 * */
public abstract class FieldSimulation {
    private final MapleCompetitionField competitionField;
    private final FieldObstaclesMap obstaclesMap;

    public FieldSimulation(MapleCompetitionField competitionField, FieldObstaclesMap obstaclesMap) {
        this.competitionField = competitionField;
        this.obstaclesMap = obstaclesMap;
    }

    /**
     * stores the obstacles on a competition field, which includes the border and the game pieces
     * */
    public static abstract class FieldObstaclesMap {
        private final List<Body> obstacles = new ArrayList<>();

        protected void addBorderLine(Translation2d startingPoint, Translation2d endingPoint) {
            final Body obstacle = getObstacle(Geometry.createSegment(
                    GeometryConvertor.toDyn4jVector2(startingPoint),
                    GeometryConvertor.toDyn4jVector2(endingPoint)
            ));
            obstacles.add(obstacle);
        }

        protected void addRectangularObstacle(double width, double height, Pose2d pose) {
            final Body obstacle = getObstacle(Geometry.createRectangle(
                    width, height
            ));

            obstacle.getTransform().set(GeometryConvertor.toDyn4jTransform(pose));
            obstacles.add(obstacle);
        }

        private static Body getObstacle(Convex shape) {
            final Body obstacle = new Body();
            obstacle.setMass(MassType.INFINITE);
            final BodyFixture fixture = obstacle.addFixture(shape);
            fixture.setFriction(0.8);
            fixture.setRestitution(0.6);
            return obstacle;
        }
    }
}

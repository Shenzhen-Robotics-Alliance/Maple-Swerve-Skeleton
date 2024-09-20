package frc.robot.utils.CompetitionFieldUtils.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CustomMaths.GeometryConvertor;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;

/**
 * simulates the behavior of gamepiece on field.
 * game pieces HAVE collision spaces.
 * they can also be "grabbed" by an Intake Simulation
 * the game piece will also be displayed on advantage scope (once registered in CompetitionFieldSimulation)
 * */
public abstract class GamePieceInSimulation extends Body implements GamePieceOnFieldDisplay {
    /**
     * for convenience, we assume all game pieces have the following properties
     * */
    public static final double
            DEFAULT_MASS = 0.2,
            LINEAR_DAMPING = 3.5,
            ANGULAR_DAMPING = 5,
            EDGE_COEFFICIENT_OF_FRICTION = 0.8,
            EDGE_COEFFICIENT_OF_RESTITUTION = 0.3;

    public GamePieceInSimulation(Translation2d initialPosition, Convex shape) {
        this(initialPosition, shape, DEFAULT_MASS);
    }

    public GamePieceInSimulation(Translation2d initialPosition, Convex shape, double mass) {
        super();
        BodyFixture bodyFixture = super.addFixture(shape);
        bodyFixture.setFriction(EDGE_COEFFICIENT_OF_FRICTION);
        bodyFixture.setRestitution(EDGE_COEFFICIENT_OF_RESTITUTION);
        bodyFixture.setDensity(mass / shape.getArea());
        super.setMass(MassType.NORMAL);

        super.translate(GeometryConvertor.toDyn4jVector2(initialPosition));

        super.setLinearDamping(LINEAR_DAMPING);
        super.setAngularDamping(ANGULAR_DAMPING);
        super.setBullet(true);
    }

    @Override
    public Pose2d getObjectOnFieldPose2d() {
        return GeometryConvertor.toWpilibPose2d(super.getTransform());
    }
}

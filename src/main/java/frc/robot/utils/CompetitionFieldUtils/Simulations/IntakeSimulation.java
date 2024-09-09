package frc.robot.utils.CompetitionFieldUtils.Simulations;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CompetitionFieldUtils.Objects.GamePieceInSimulation;
import org.dyn4j.collision.CollisionBody;
import org.dyn4j.collision.Fixture;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.contact.Contact;
import org.dyn4j.dynamics.contact.SolvedContact;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Segment;
import org.dyn4j.world.ContactCollisionData;
import org.dyn4j.world.listener.ContactListener;

import java.util.ArrayDeque;
import java.util.Queue;

import static frc.robot.utils.CustomMaths.GeometryConvertor.*;

public abstract class IntakeSimulation extends BodyFixture {
    private final int capacity;
    protected int gamePieceCount;

    private final Queue<GamePieceInSimulation> gamePiecesToRemove;

    /**
     * Creates an intake simulation
     * the intake is considered a line segment on the robot
     * any game piece that touches the line will be grabbed
     *
     * @param startPointOnRobot     the start point of the segment, in relative to the robot
     * @param endPointOnRobot       the end point of the segment, in relative to the robot
     * @param capacity              the amount of game-pieces that can be hold in the intake
     */
    public IntakeSimulation(Translation2d startPointOnRobot, Translation2d endPointOnRobot, int capacity) {
        this(
                new Segment(toDyn4jVector2(startPointOnRobot), toDyn4jVector2(endPointOnRobot)),
                capacity
        );
    }

    /**
     * Creates an intake simulation
     * the intake is fixed shape on the robot
     * any game piece that touches the line will be grabbed
     *
     * @param shape the shape of the intake
     * @param capacity              the amount of game-pieces that can be hold in the intake\
     */
    public IntakeSimulation(Convex shape, int capacity) {
        super(shape);
        if (capacity > 100)
            throw new IllegalArgumentException("capacity too large, max is 100");
        this.capacity = capacity;

        this.gamePiecesToRemove = new ArrayDeque<>(capacity);
    }

    protected abstract boolean isIntakeRunning();

    public final class GamePieceContactListener implements ContactListener<Body> {
        @Override
        public void begin(ContactCollisionData collision, Contact contact) {
            if (!isIntakeRunning()) return;
            if (gamePieceCount == capacity) return;

            final CollisionBody<?> collisionBody1 = collision.getBody1(), collisionBody2 = collision.getBody2();
            final Fixture fixture1 = collision.getFixture1(), fixture2 = collision.getFixture2();

            if (collisionBody1 instanceof GamePieceInSimulation && fixture2 == IntakeSimulation.this)
                flagGamePieceForRemoval((GamePieceInSimulation) collisionBody1);
            else if (collisionBody2 instanceof GamePieceInSimulation && fixture1 == IntakeSimulation.this)
                flagGamePieceForRemoval((GamePieceInSimulation) collisionBody2);
        }

        private void flagGamePieceForRemoval(GamePieceInSimulation gamePiece) {
            gamePiecesToRemove.add(gamePiece);
            gamePieceCount++;
        }

        /* functions not used */
        @Override public void persist(ContactCollisionData collision, Contact oldContact, Contact newContact) {}
        @Override public void end(ContactCollisionData collision, Contact contact) {}
        @Override public void destroyed(ContactCollisionData collision, Contact contact) {}
        @Override public void collision(ContactCollisionData collision) {}
        @Override public void preSolve(ContactCollisionData collision, Contact contact) {}
        @Override public void postSolve(ContactCollisionData collision, SolvedContact contact) {}
    }

    public GamePieceContactListener getGamePieceContactListener() {
        return new GamePieceContactListener();
    }

    public Queue<GamePieceInSimulation> getGamePiecesToRemove() {
        return gamePiecesToRemove;
    }

    public void clearGamePiecesToRemoveQueue() {
        this.gamePieceCount += gamePiecesToRemove.size();
        gamePiecesToRemove.clear();
    }
}

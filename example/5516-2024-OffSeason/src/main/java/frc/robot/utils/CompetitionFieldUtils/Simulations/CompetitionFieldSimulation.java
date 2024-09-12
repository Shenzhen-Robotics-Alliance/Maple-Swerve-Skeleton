package frc.robot.utils.CompetitionFieldUtils.Simulations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Robot;
import frc.robot.constants.LogPaths;
import frc.robot.utils.CompetitionFieldUtils.Objects.GamePieceInSimulation;
import frc.robot.utils.CompetitionFieldUtils.CompetitionFieldVisualizer;
import frc.robot.utils.CustomMaths.GeometryConvertor;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static frc.robot.constants.DriveTrainConstants.*;

/**
 * this class simulates the physical behavior of all the objects on field
 * should only be created during a robot simulation (not in real or replay mode)
 * */
public abstract class CompetitionFieldSimulation {
    protected final World<Body> physicsWorld;
    protected final CompetitionFieldVisualizer competitionField;
    protected final Set<HolonomicChassisSimulation> robotSimulations = new HashSet<>();
    protected final HolonomicChassisSimulation mainRobot;
    protected final Set<GamePieceInSimulation> gamePieces;

    private List<IntakeSimulation> intakeSimulations = new ArrayList<>();

    public CompetitionFieldSimulation(HolonomicChassisSimulation mainRobot, FieldObstaclesMap obstaclesMap) {
        this.competitionField = new CompetitionFieldVisualizer(mainRobot);
        this.mainRobot = mainRobot;
        this.physicsWorld = new World<>();
        this.physicsWorld.setGravity(PhysicsWorld.ZERO_GRAVITY);
        for (Body obstacle: obstaclesMap.obstacles)
            this.physicsWorld.addBody(obstacle);
        this.gamePieces = new HashSet<>();

        this.physicsWorld.addBody(mainRobot);
        this.robotSimulations.add(mainRobot);
    }

    public void updateSimulationWorld() {
        final long t0 = System.nanoTime();
        competitionPeriodic();
        final double subPeriodSeconds = Robot.defaultPeriodSecs / SIMULATION_TICKS_IN_1_PERIOD;
        // move through 5 sub-periods in each update
        for (int i = 0; i < SIMULATION_TICKS_IN_1_PERIOD; i++) {
            this.physicsWorld.step(1, subPeriodSeconds);
            for (HolonomicChassisSimulation robotSimulation:robotSimulations)
                robotSimulation.updateSimulationSubTick(i, subPeriodSeconds);
        }

        for (IntakeSimulation intakeSimulation:intakeSimulations)
            while (!intakeSimulation.getGamePiecesToRemove().isEmpty())
                this.removeGamePiece(intakeSimulation.getGamePiecesToRemove().poll());

        Logger.recordOutput(
                LogPaths.PHYSICS_SIMULATION_PATH + "dyn4j simulator time millis",
                (System.nanoTime() - t0) / 1000000.0
        );
    }

    public void addRobot(HolonomicChassisSimulation chassisSimulation) {
        this.physicsWorld.addBody(chassisSimulation);
        this.robotSimulations.add(chassisSimulation);
        this.competitionField.addObject(chassisSimulation);
    }

    public void registerIntake(IntakeSimulation intakeSimulation) {
        this.intakeSimulations.add(intakeSimulation);
        this.mainRobot.addFixture(intakeSimulation);
        this.physicsWorld.addContactListener(intakeSimulation.getGamePieceContactListener());
    }

    public void addGamePiece(GamePieceInSimulation gamePieceInSimulation) {
        this.physicsWorld.addBody(gamePieceInSimulation);
        this.competitionField.addObject(gamePieceInSimulation);
        this.gamePieces.add(gamePieceInSimulation);
    }

    public void removeGamePiece(GamePieceInSimulation gamePieceInSimulation) {
        this.physicsWorld.removeBody(gamePieceInSimulation);
        this.competitionField.deleteObject(gamePieceInSimulation);
        this.gamePieces.remove(gamePieceInSimulation);
    }

    public CompetitionFieldVisualizer getVisualizer() {return competitionField;}
    public HolonomicChassisSimulation getMainRobot() {return mainRobot;}

    public void clearGamePieces() {
        for (GamePieceInSimulation gamePiece: this.gamePieces) {
            this.physicsWorld.removeBody(gamePiece);
            this.competitionField.clearObjectsWithGivenType(gamePiece.getTypeName());
        }
        this.gamePieces.clear();
    }

    public void resetFieldForAuto() {
        clearGamePieces();
        placeGamePiecesOnField();
    }

    /**
     * do field reset by placing all the game-pieces on field(for autonomous)
     * */
    public abstract void placeGamePiecesOnField();

    /**
     * update the score counts & human players periodically
     * implement this method in current year's simulation
     * */
    public abstract void competitionPeriodic();

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
            fixture.setFriction(0.6);
            fixture.setRestitution(0.6);
            return obstacle;
        }
    }
}

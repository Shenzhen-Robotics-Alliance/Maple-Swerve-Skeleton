package frc.robot.utils.CompetitionFieldUtils.Objects;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.Simulations.CompetitionFieldSimulation;
import frc.robot.utils.CustomMaths.GeometryConvertor;
import frc.robot.utils.MapleTimeUtils;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;

import static frc.robot.constants.FieldConstants.FIELD_WIDTH;

/**
 * a set of game pieces of the 2024 game "Crescendo"
 * */
public final class Crescendo2024FieldObjects {
    /* https://www.andymark.com/products/frc-2024-am-4999 */
     private static final double
            NOTE_HEIGHT = Units.inchesToMeters(2),
            NOTE_DIAMETER = Units.inchesToMeters(14);

    /**
     * a static note on field
     * it is displayed on the dashboard and telemetry,
     * but it does not appear in the simulation.
     * meaning that, it does not have collision space and isn't involved in intake simulation
     * */
    public static class NoteOnFieldStatic implements GamePieceOnFieldDisplay {
        private final Translation2d initialPosition;

        public NoteOnFieldStatic(Translation2d initialPosition) {
            this.initialPosition = initialPosition;
        }

        @Override
        public Pose2d getObjectOnFieldPose2d() {
            return new Pose2d(initialPosition, new Rotation2d());
        }

        @Override
        public String getTypeName() {
            return "Note";
        }

        @Override
        public double getGamePieceHeight() {
            return NOTE_HEIGHT;
        }
    }

    /**
     * a simulated note on field
     * has collision space, and can be "grabbed" by an intake simulation
     * */
    public static class NoteOnFieldSimulated extends GamePieceInSimulation {
        public NoteOnFieldSimulated(Translation2d initialPosition) {
            super(initialPosition, Geometry.createCircle(NOTE_DIAMETER/2));
        }

        @Override
        public double getGamePieceHeight() {
            return NOTE_HEIGHT;
        }

        @Override
        public String getTypeName() {
            return "Note";
        }
    }

    /**
     * a note that is flying from a shooter to the speaker
     * the flight is simulated by a simple linear animation
     * */
    public static class NoteFlyingToSpeaker extends GamePieceOnFlyDisplay {
        public NoteFlyingToSpeaker(Translation3d shooterPosition, double flightTimeSeconds) {
            this(shooterPosition, flightTimeSeconds, false);
        }
        public NoteFlyingToSpeaker(Translation3d shooterPosition, double flightTimeSeconds, boolean reverseSide) {
            super(
                    shooterPosition,
                    FieldConstants.toCurrentAllianceTranslation(
                            reverseSide ? new Translation3d(
                                    FIELD_WIDTH - FieldConstants.SPEAKER_POSE_BLUE.getX(),
                                    FieldConstants.SPEAKER_POSE_BLUE.getY(),
                                    FieldConstants.SPEAKER_POSE_BLUE.getZ())
                                    : FieldConstants.SPEAKER_POSE_BLUE
                    ),
                    flightTimeSeconds
            );
        }

        @Override
        public String getTypeName() {
            return "Note";
        }
    }

    private static final double
            FEED_SHOT_INITIAL_HEIGHT_METERS = 0.3,
            FEED_SHOT_VERTICAL_SPEED_MPS = 7.5,
            /*
            * root of equation:
            * h0 + v0 * t - 1/2 * g * t^2 = 0
            * solve for t = ...
            *  */
            FEED_SHOT_FLIGHT_TIME_SECONDS = 1.538;
    /**
     * a feed-shot note
     * has collision space
     * */
    public static final class FeedShotHighNote extends GamePieceOnFlyDisplay {
        private final Rotation2d robotFacingWhenLaunching;
        public FeedShotHighNote(Translation2d shooterPositionOnField, Translation2d targetedPositionOnField, CompetitionFieldSimulation simulation) {
            super(
                    new Translation3d(shooterPositionOnField.getX(), shooterPositionOnField.getY(), 0),
                    getNoteTouchGroundTranslation3d(targetedPositionOnField),
                    FEED_SHOT_FLIGHT_TIME_SECONDS
            );
            this.robotFacingWhenLaunching = targetedPositionOnField.minus(shooterPositionOnField).getAngle();
            final Command addNoteOnGroundToSimulator = Commands.runOnce(() -> {
                final Crescendo2024FieldObjects.NoteOnFieldSimulated note =  new Crescendo2024FieldObjects.NoteOnFieldSimulated(
                        targetedPositionOnField
                );
                note.setLinearVelocity(GeometryConvertor.toDyn4jVector2(new Translation2d(
                        shooterPositionOnField.getDistance(targetedPositionOnField) / FEED_SHOT_FLIGHT_TIME_SECONDS,
                        robotFacingWhenLaunching
                )));
                simulation.addGamePiece(note);
            });
            if (simulation != null) CommandScheduler.getInstance().schedule(
                    Commands.waitSeconds(FEED_SHOT_FLIGHT_TIME_SECONDS).andThen(addNoteOnGroundToSimulator)
            );
        }

        private static Translation3d getNoteTouchGroundTranslation3d(Translation2d noteEndingTranslation2d) {
            return new Translation3d(
                    noteEndingTranslation2d.getX(),
                    noteEndingTranslation2d.getY(),
                    0.2
            );
        }

        @Override
        public String getTypeName() {
            return "Note";
        }

        @Override
        public Pose3d getPose3d() {
            final double t = getTimeSinceLaunchSeconds(),
                    height = FEED_SHOT_INITIAL_HEIGHT_METERS
                            + FEED_SHOT_VERTICAL_SPEED_MPS * t
                            - 1.0/2.0 * t * t * 10;
            return new Pose3d(
                    new Translation3d(super.getPose3d().getX(), super.getPose3d().getY(), height),
                    new Rotation3d(0, Math.toRadians(-35), robotFacingWhenLaunching.getRadians())
            );
        }
    }

    /**
     * a feed-shot note
     * has collision space
     * */
    private static final double
            LOW_SHOT_NOTE_SPEED_MPS = 8,
            LOW_SHOT_VERTICAL_SPEED_MPS = 3,
            LOW_SHOT_INITIAL_HEIGHT = 0.3;
    public static final class FeedShotLowNote extends GamePieceInSimulation {
        private final double launchTimeSeconds;
        public FeedShotLowNote(Translation2d initialPosition, Rotation2d robotFacingWhenLaunching) {
            super(initialPosition, Geometry.createCircle(NOTE_DIAMETER/2));
            setLinearDamping(0);
            setLinearVelocity(GeometryConvertor.toDyn4jVector2(
                    new Translation2d(LOW_SHOT_NOTE_SPEED_MPS, 0)
                            .rotateBy(robotFacingWhenLaunching)
            ));
            this.launchTimeSeconds = MapleTimeUtils.getLogTimeSeconds();
            CommandScheduler.getInstance().schedule(
                    Commands.waitUntil(() -> getGamePieceHeight() == NOTE_HEIGHT)
                            .andThen(() -> setLinearDamping(GamePieceInSimulation.LINEAR_DAMPING))
            );
        }


        @Override
        public String getTypeName() {
            return "Note";
        }

        @Override
        public double getGamePieceHeight() {
            final double t = MapleTimeUtils.getLogTimeSeconds() - launchTimeSeconds,
                    height = LOW_SHOT_INITIAL_HEIGHT
                            + LOW_SHOT_VERTICAL_SPEED_MPS * t
                            - 1.0/2.0 * t * t * 10;
            return Math.max(NOTE_HEIGHT, height);
        }
    }
}

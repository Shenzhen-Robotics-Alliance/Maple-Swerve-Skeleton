package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;
import org.dyn4j.geometry.Geometry;

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
    public static class NoteOnFly extends GamePieceOnFlyDisplay {
        public NoteOnFly(Translation3d shooterPosition, double flightTimeSeconds) {
            super(
                    shooterPosition,
                    Constants.toCurrentAllianceTranslation(Constants.CrescendoField2024Constants.SPEAKER_POSE_BLUE),
                    flightTimeSeconds
            );
        }

        @Override
        public String getTypeName() {
            return "Note";
        }
    }
}

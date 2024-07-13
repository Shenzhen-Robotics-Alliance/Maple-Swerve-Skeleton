package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;
import org.dyn4j.geometry.Geometry;

public final class Crescendo2024FieldObjects {
     private static final double
            NOTE_HEIGHT = Units.inchesToMeters(2),
            NOTE_DIAMETER = Units.inchesToMeters(14);

    /**
     * a static note on field
     * it can be displayed
     * but do not have collision space in simulation
     * */
    public static class NoteOnFieldStatic implements GamePieceOnField {
        private final Translation2d initialPosition;

        public NoteOnFieldStatic(Translation2d initialPosition) {
            this.initialPosition = initialPosition;
        }

        @Override
        public Pose2d getPose2d() {
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

    public static class NoteOnFly implements MapleCompetitionField.ObjectOnField {
        private final double launchingTimeStampSec, launchingHeightMeters, launchingSpeedMetersPerSec;
        private final Translation2d positionLaunched;

        public NoteOnFly(double launchingTimeStampSec, double launchingHeightMeters, double launchingSpeedMetersPerSec, Translation2d positionLaunched) {
            this.launchingTimeStampSec = launchingTimeStampSec;
            this.launchingHeightMeters = launchingHeightMeters;
            this.launchingSpeedMetersPerSec = launchingSpeedMetersPerSec;
            this.positionLaunched = positionLaunched;
        }

        @Override
        public String getTypeName() {
            return "Note";
        }

        @Override
        public Pose3d getPose3d() {
            return null; // TODO linear interpret the position when the note is on fly
        }
    }
}

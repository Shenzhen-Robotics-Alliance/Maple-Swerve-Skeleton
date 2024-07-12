package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import edu.wpi.first.math.geometry.*;
import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;

public final class Crescendo2024FieldObjects {
    public interface NoteOnField extends MapleCompetitionField.ObjectOn2dField {
        double NOTE_HEIGHT_METERS = 0.05, NOTE_SQUEEZED_METERS = 0.005;
        @Override
        default String getTypeName() {
            return "Note";
        }

        @Override
        default Pose3d getPose3d() {
            final Pose2d pose2d = getPose2d();
            final Translation3d translation3d = new Translation3d(
                    pose2d.getX(),
                    pose2d.getY(),
                    NOTE_HEIGHT_METERS/2 - NOTE_SQUEEZED_METERS
            );
            return new Pose3d(translation3d, new Rotation3d());
        }
    }

    public static class NoteOnFieldStatic implements NoteOnField {
        private final Pose2d pose2d;

        public NoteOnFieldStatic(Translation2d initialPosition) {
            this.pose2d = new Pose2d(initialPosition, new Rotation2d());
        }

        @Override
        public Pose2d getPose2d() {
            return pose2d;
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

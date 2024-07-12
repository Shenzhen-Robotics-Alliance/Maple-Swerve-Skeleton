package frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;

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

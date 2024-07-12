package frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NoteOnFieldStatic implements NoteOnField {
    private final Pose2d pose2d;

    public NoteOnFieldStatic(Translation2d initialPosition) {
        this.pose2d = new Pose2d(initialPosition, new Rotation2d());
    }

    @Override
    public Pose2d getPose2d() {
        return pose2d;
    }
}

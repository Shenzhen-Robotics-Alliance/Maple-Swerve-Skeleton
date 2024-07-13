package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import edu.wpi.first.math.geometry.*;
import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;

public interface GamePieceOnField extends MapleCompetitionField.ObjectOn2dField {
    @Override
    default Pose3d getPose3d() {
        final Pose2d pose2d = getPose2d();
        final Translation3d translation3d = new Translation3d(
                pose2d.getX(),
                pose2d.getY(),
                getGamePieceHeight()/2
        );
        return new Pose3d(translation3d, new Rotation3d());
    }

    /**
     * @return the height of the game piece when standing from ground, in meters
     * */
    double getGamePieceHeight();
}

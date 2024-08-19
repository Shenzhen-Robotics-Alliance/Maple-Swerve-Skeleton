package frc.robot.utils.CompetitionFieldUtils.Objects;

import edu.wpi.first.math.geometry.*;
import frc.robot.utils.CompetitionFieldUtils.CompetitionFieldVisualizer;

/**
 * displays a game piece on field to Advantage Scope
 * since game pieces MUST be displayed as 3d objects in Advantage Scope
 * we have to convert the 2d pose of the game piece to a 3d pose
 * */
public interface GamePieceOnFieldDisplay extends CompetitionFieldVisualizer.Object2dOnFieldDisplay {
    @Override
    default Pose3d getPose3d() {
        final Pose2d pose2d = getObjectOnFieldPose2d();
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

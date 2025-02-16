package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import java.io.IOException;
import org.ironmaple.utils.FieldMirroringUtils;
import org.json.simple.parser.ParseException;

public interface Auto {
    Command getAutoCommand(RobotContainer robot) throws IOException, ParseException;

    Pose2d getStartingPoseAtBlueAlliance();

    static Auto none() {
        return new Auto() {
            @Override
            public Command getAutoCommand(RobotContainer robot) {
                return Commands.none();
            }

            @Override
            public Pose2d getStartingPoseAtBlueAlliance() {
                return new Pose2d(3, 3, new Rotation2d());
            }
        };
    }

    static PathPlannerPath getChoreoPath(String name, boolean mirror) throws IOException, ParseException {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
        return mirror ? path.mirrorPath() : path;
    }

    static Pose2d flipLeftRight(Pose2d pose) {
        return new Pose2d(
                pose.getX(),
                FieldMirroringUtils.FIELD_HEIGHT - pose.getY(),
                pose.getRotation().unaryMinus());
    }
}

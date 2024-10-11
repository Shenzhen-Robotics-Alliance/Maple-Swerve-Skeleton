package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import org.json.simple.parser.ParseException;

import java.io.IOException;

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
}

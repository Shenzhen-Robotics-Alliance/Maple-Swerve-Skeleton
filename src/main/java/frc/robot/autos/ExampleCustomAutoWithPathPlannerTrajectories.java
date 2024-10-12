package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class ExampleCustomAutoWithPathPlannerTrajectories implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.none();
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(1.3, 7.2, Rotation2d.fromDegrees(180));
    }
}

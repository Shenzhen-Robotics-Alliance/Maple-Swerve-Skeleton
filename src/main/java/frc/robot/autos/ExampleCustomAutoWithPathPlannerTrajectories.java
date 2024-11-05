package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class ExampleCustomAutoWithPathPlannerTrajectories implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("rush first pp")));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("rush second pp")));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("rush third pp")));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("rush fourth and shoot pp")));

        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(1.49, 7.65, Rotation2d.fromDegrees(180));
    }
}

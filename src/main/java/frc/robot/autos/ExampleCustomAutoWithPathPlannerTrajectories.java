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

        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("place first")));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("grab second")));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("place second")));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("grab third")));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("place third")));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("grab fourth")));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("score fourth")));

        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(7.85, 6.17, Rotation2d.fromDegrees(180));
    }
}

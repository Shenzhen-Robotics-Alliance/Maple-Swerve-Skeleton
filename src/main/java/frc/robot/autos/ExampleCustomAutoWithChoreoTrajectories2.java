package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class ExampleCustomAutoWithChoreoTrajectories2 implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("shoot preload and second")).asProxy()
        );
        commandGroup.addCommands(
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("shoot third and fourth")).asProxy()
        );
        commandGroup.addCommands(
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab fifth and shoot")).asProxy()
        );

        commandGroup.addCommands(
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab sixth and shoot")).asProxy()
        );

        commandGroup.addCommands(
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab seven and shoot")).asProxy()
        );

        commandGroup.addCommands(
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab eighth and shoot")).asProxy()
        );
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(1.46,4.6, Rotation2d.fromDegrees(180));
    }
}

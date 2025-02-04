package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.reefscape.ReefAlignment;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class ExampleCustomAutoWithChoreoTrajectories2 implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, PathPlannerPath.fromChoreoTrajectory("auto2 - place first"), 8, Commands::none));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("auto2 - grab second"))
                .asProxy());

        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, PathPlannerPath.fromChoreoTrajectory("auto2 - place second"), 9, Commands::none));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("auto2 - grab third"))
                .asProxy());

        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, PathPlannerPath.fromChoreoTrajectory("auto2 - place third"), 10, Commands::none));

        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("auto2 - grab fourth"))
                .asProxy());

        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, PathPlannerPath.fromChoreoTrajectory("auto2 - place fourth"), 11, Commands::none));
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(7.843, 6.16, Rotation2d.fromDegrees(180));
    }
}

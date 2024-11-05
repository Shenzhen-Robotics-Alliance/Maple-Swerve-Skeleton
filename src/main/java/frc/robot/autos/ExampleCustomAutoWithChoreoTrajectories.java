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

public class ExampleCustomAutoWithChoreoTrajectories implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("rush first"))
                .asProxy());
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("rush second"))
                .asProxy());
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("rush third"))
                .asProxy());

        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("rush fourth and shoot"))
                .asProxy());
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(1.42, 7.8, Rotation2d.fromDegrees(180));
    }
}

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
    private final boolean isRightSide;

    public ExampleCustomAutoWithChoreoTrajectories2(boolean isRightSide) {
        this.isRightSide = isRightSide;
    }

    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, Auto.getChoreoPath("auto2 - place first", isRightSide), isRightSide ? 5 : 8, Commands.none()));
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("auto2 - grab second"))
                .asProxy());

        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, Auto.getChoreoPath("auto2 - place second", isRightSide), isRightSide ? 4:9, Commands.none()));
        commandGroup.addCommands(AutoBuilder.followPath(Auto.getChoreoPath("auto2 - grab third", isRightSide))
                .asProxy());

        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, Auto.getChoreoPath("auto2 - place third", isRightSide), isRightSide ? 3 : 10, Commands.none()));

        commandGroup.addCommands(AutoBuilder.followPath(Auto.getChoreoPath("auto2 - grab fourth", isRightSide))
                .asProxy());

        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot,Auto.getChoreoPath("auto2 - place fourth", isRightSide), 11, Commands.none()));
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        Pose2d startingPoseAtLeft = new Pose2d(7.843, 6.16, Rotation2d.fromDegrees(180));
        return isRightSide ? Auto.flipLeftRight(startingPoseAtLeft) : startingPoseAtLeft;
    }
}

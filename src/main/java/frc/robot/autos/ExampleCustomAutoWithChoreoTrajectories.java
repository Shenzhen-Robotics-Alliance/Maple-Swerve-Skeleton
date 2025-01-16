package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AutoAlignment;
import java.io.IOException;
import java.util.OptionalInt;
import org.ironmaple.utils.FieldMirroringUtils;
import org.json.simple.parser.ParseException;

public class ExampleCustomAutoWithChoreoTrajectories implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromChoreoTrajectory("place first"),
                        () -> FieldMirroringUtils.toCurrentAlliancePose(
                                new Pose2d(5.27, 5.02, Rotation2d.fromDegrees(-120))),
                        () -> FieldMirroringUtils.isSidePresentedAsRed() ? OptionalInt.of(11) : OptionalInt.of(20))
                .asProxy());
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab second"))
                .asProxy());
        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromChoreoTrajectory("place second"),
                        () -> FieldMirroringUtils.toCurrentAlliancePose(
                                new Pose2d(4.99, 5.17, Rotation2d.fromDegrees(-120))),
                        () -> FieldMirroringUtils.isSidePresentedAsRed() ? OptionalInt.of(11) : OptionalInt.of(20))
                .asProxy());
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab third"))
                .asProxy());

        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromChoreoTrajectory("place third"),
                        () -> FieldMirroringUtils.toCurrentAlliancePose(
                                new Pose2d(3.99, 5.19, Rotation2d.fromDegrees(-60))),
                        () -> FieldMirroringUtils.isSidePresentedAsRed() ? OptionalInt.of(6) : OptionalInt.of(19))
                .asProxy());

        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab fourth"))
                .asProxy());

        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromChoreoTrajectory("place fourth"),
                        () -> FieldMirroringUtils.toCurrentAlliancePose(new Pose2d(3.24, 4.19, Rotation2d.kZero)),
                        () -> FieldMirroringUtils.isSidePresentedAsRed() ? OptionalInt.of(7) : OptionalInt.of(18))
                .asProxy());
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(7.78, 6.13, Rotation2d.fromDegrees(180));
    }
}

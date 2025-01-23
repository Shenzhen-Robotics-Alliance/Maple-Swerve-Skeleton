package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AutoAlignment;
import frc.robot.constants.DriveControlLoops;
import java.io.IOException;
import java.util.OptionalInt;
import org.ironmaple.utils.FieldMirroringUtils;
import org.json.simple.parser.ParseException;

public class ExampleCustomAutoWithChoreoTrajectories2 implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromChoreoTrajectory("auto2 - place first"),
                        () -> FieldMirroringUtils.toCurrentAlliancePose(
                                new Pose2d(5.27, 5.02, Rotation2d.fromDegrees(-120))),
                        () -> FieldMirroringUtils.isSidePresentedAsRed() ? OptionalInt.of(11) : OptionalInt.of(20),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG)
                .asProxy());
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("auto2 - grab second"))
                .asProxy());

        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromChoreoTrajectory("auto2 - place second"),
                        () -> FieldMirroringUtils.toCurrentAlliancePose(
                                new Pose2d(5.00, 5.17, Rotation2d.fromDegrees(-120))),
                        () -> FieldMirroringUtils.isSidePresentedAsRed() ? OptionalInt.of(11) : OptionalInt.of(20),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG)
                .asProxy());
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("auto2 - grab third"))
                .asProxy());

        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromChoreoTrajectory("auto2 - place third"),
                        () -> FieldMirroringUtils.toCurrentAlliancePose(
                                new Pose2d(3.99, 5.19, Rotation2d.fromDegrees(-60))),
                        () -> FieldMirroringUtils.isSidePresentedAsRed() ? OptionalInt.of(6) : OptionalInt.of(19),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG)
                .asProxy());

        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("auto2 - grab fourth"))
                .asProxy());

        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromChoreoTrajectory("auto2 - place fourth"),
                        () -> FieldMirroringUtils.toCurrentAlliancePose(
                                new Pose2d(3.73, 5.03, Rotation2d.fromDegrees(-60))),
                        () -> FieldMirroringUtils.isSidePresentedAsRed() ? OptionalInt.of(6) : OptionalInt.of(19),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG)
                .asProxy());
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(7.843, 6.16, Rotation2d.fromDegrees(180));
    }
}

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AutoAlignment;
import frc.robot.constants.DriveControlLoops;
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
                        FieldMirroringUtils.toCurrentAlliancePose(new Pose2d(5.27, 5.02, Rotation2d.fromDegrees(-120))),
                        Rotation2d.fromDegrees(-120),
                        OptionalInt.of(20),
                        OptionalInt.of(11),
                        OptionalInt.of(1),
                        Commands.none(),
                        Commands.none(),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG_AUTONOMOUS)
                .asProxy());
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab second"))
                .asProxy());
        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromChoreoTrajectory("place second"),
                        FieldMirroringUtils.toCurrentAlliancePose(new Pose2d(5.00, 5.17, Rotation2d.fromDegrees(-120))),
                        Rotation2d.fromDegrees(-120),
                        OptionalInt.of(20),
                        OptionalInt.of(11),
                        OptionalInt.of(0),
                        Commands.none(),
                        Commands.none(),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG_AUTONOMOUS)
                .asProxy());
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab third"))
                .asProxy());

        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromChoreoTrajectory("place third"),
                        FieldMirroringUtils.toCurrentAlliancePose(new Pose2d(3.99, 5.19, Rotation2d.fromDegrees(-60))),
                        Rotation2d.fromDegrees(-60),
                        OptionalInt.of(19),
                        OptionalInt.of(6),
                        OptionalInt.of(1),
                        Commands.none(),
                        Commands.none(),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG_AUTONOMOUS)
                .asProxy());

        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab fourth"))
                .asProxy());

        //        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
        //                        robot.drive,
        //                        robot.aprilTagVision,
        //                        PathPlannerPath.fromChoreoTrajectory("place fourth center"),
        //                        () -> FieldMirroringUtils.toCurrentAlliancePose(new Pose2d(3.24, 4.19,
        // Rotation2d.kZero)),
        //                        () -> FieldMirroringUtils.isSidePresentedAsRed() ? OptionalInt.of(7) :
        // OptionalInt.of(18))
        //                .asProxy());
        commandGroup.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromChoreoTrajectory("place fourth"),
                        FieldMirroringUtils.toCurrentAlliancePose(new Pose2d(3.73, 5.03, Rotation2d.fromDegrees(-60))),
                        Rotation2d.fromDegrees(-60),
                        OptionalInt.of(19),
                        OptionalInt.of(6),
                        OptionalInt.of(0),
                        Commands.none(),
                        Commands.none(),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG_AUTONOMOUS)
                .asProxy());
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(7.78, 6.13, Rotation2d.fromDegrees(180));
    }
}

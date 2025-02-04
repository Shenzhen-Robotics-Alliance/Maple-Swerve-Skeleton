package frc.robot.autos;

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

public class ExampleCustomAutoWithAutoAlignment implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addCommands(AutoAlignment.followPathAndAutoAlign(
                        robot.drive,
                        robot.aprilTagVision,
                        PathPlannerPath.fromPathFile("Test Auto Alignment"),
                        FieldMirroringUtils.toCurrentAlliancePose(new Pose2d(4.98, 5.18, Rotation2d.fromDegrees(-120))),
                        Rotation2d.fromDegrees(-120),
                        OptionalInt.of(20),
                        OptionalInt.of(11),
                        OptionalInt.of(0),
                        Commands.none(),
                        Commands.none(),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG_AUTONOMOUS)
                .asProxy());
        return sequence;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(7.678, 6.932, Rotation2d.k180deg);
    }
}

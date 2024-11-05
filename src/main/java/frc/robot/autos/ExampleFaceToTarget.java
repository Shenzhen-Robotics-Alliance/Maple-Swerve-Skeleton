package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.FollowPathFaceToTarget;
import java.io.IOException;
import org.ironmaple.utils.FieldMirroringUtils;
import org.json.simple.parser.ParseException;

public class ExampleFaceToTarget implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return FollowPathFaceToTarget.followPathFacetToTarget(
                PathPlannerPath.fromPathFile("Test Face To Target"),
                0,
                FieldMirroringUtils.SPEAKER_POSITION_SUPPLIER,
                null);
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(2, 7, Rotation2d.fromDegrees(-90));
    }
}

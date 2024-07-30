package frc.robot.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.RobotOnFieldDisplay;
import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;
import org.littletonrobotics.junction.Logger;

public class FieldDisplayTest extends Command {
    private MapleCompetitionField field;

    @Override
    public void initialize() {
        final RobotOnFieldDisplay mainRobot = () -> new Pose2d(4, 4, new Rotation2d()),
                anotherRobot = () -> new Pose2d(4, 5, new Rotation2d());
        field = new MapleCompetitionField(mainRobot);
        field.addObject(anotherRobot);

        field.addObject(new Crescendo2024FieldObjects.NoteOnFieldStatic(new Translation2d( 2, 1)));

        PathPlannerPath path = PathPlannerPath.fromPathFile("opponent cycle path 2");
        Logger.recordOutput("Test Path", path.getPathPoses().toArray(Pose2d[]::new));
        Logger.recordOutput("Test Path distance", path.getAllPathPoints().stream().mapToDouble((pathPoint -> pathPoint.distanceAlongPath)).toArray());
    }

    @Override
    public void execute() {
        field.updateObjectsToDashboardAndTelemetry();
    }
}

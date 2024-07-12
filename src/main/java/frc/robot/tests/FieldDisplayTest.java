package frc.robot.tests;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.RobotOnField;
import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;

public class FieldDisplayTest implements UnitTest {
    private final MapleCompetitionField field = new MapleCompetitionField(() -> new Pose2d(3, 3, new Rotation2d()));
    @Override
    public void testStart() {
        final RobotOnField robot1 = () -> new Pose2d(4, 4, new Rotation2d()),
                robot2 = () -> new Pose2d(4, 5, new Rotation2d());
        field.addObject(robot1);
        field.addObject(robot2);
        field.addObject(new Crescendo2024FieldObjects.NoteOnFieldStatic(new Translation2d( 2, 1)));
    }

    @Override
    public void testPeriodic() {
        field.updateObjectsToDashboardAndTelemetry();
    }
}

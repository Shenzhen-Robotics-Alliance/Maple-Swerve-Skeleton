package frc.robot.tests;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.RobotOnFieldDisplay;
import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;

public class FieldDisplayTest implements UnitTest {
    private MapleCompetitionField field;
    @Override
    public void testStart() {
        final RobotOnFieldDisplay mainRobot = () -> new Pose2d(4, 4, new Rotation2d()),
                anotherRobot = () -> new Pose2d(4, 5, new Rotation2d());
        field = new MapleCompetitionField(mainRobot);
        field.addObject(anotherRobot);

        field.addObject(new Crescendo2024FieldObjects.NoteOnFieldStatic(new Translation2d( 2, 1)));
    }

    @Override
    public void testPeriodic() {
        field.updateObjectsToDashboardAndTelemetry();
    }
}

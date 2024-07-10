package frc.robot.tests;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Config.MapleInterpolationTable;

public class InterpolationTableTest implements UnitTest {
    private final MapleInterpolationTable testTable;
    public InterpolationTableTest() {
        testTable = new MapleInterpolationTable(
                "testTable",
                new MapleInterpolationTable.Variable("x", 1, 2, 3, 4),
                new MapleInterpolationTable.Variable("y", 2, 3, 4, 5)
        );
    }

    @Override
    public void testStart() {
        SmartDashboard.putNumber("InterpolationTables/testValueX", 1.0);
    }

    @Override
    public void testPeriodic() {
        final double x = SmartDashboard.getNumber("InterpolationTables/testValueX", 1.0);
        SmartDashboard.putNumber(
                "InterpolationTables/testOutPutY",
                testTable.interpolateVariable("y", x)
        );
    }
}

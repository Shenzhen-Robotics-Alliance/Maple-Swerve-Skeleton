package frc.robot.tests;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.Config.MapleInterpolationTable;

import java.io.IOException;

public class InterpolationTableTest implements UnitTest {
    private final MapleInterpolationTable testTable;
    public InterpolationTableTest() {
        try {
            testTable = MapleInterpolationTable.fromConfigFile(MapleConfigFile.fromDeployedConfig("InterpolatedMotorFeedForward", "DrivingMotorOpenLoop"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void testStart() {
        SmartDashboard.putNumber("InterpolationTables/testValueVelocity", 1.0);
    }

    @Override
    public void testPeriodic() {
        assert testTable != null;
        final double vel = SmartDashboard.getNumber("InterpolationTables/testValueVelocity", 1.0);
        SmartDashboard.putNumber(
                "InterpolationTables/testOutPutPower",
                testTable.interpolateVariable("motorPower", vel)
        );
    }
}

package frc.robot.tests;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.Config.MapleInterpolationTable;

import java.io.IOException;

public class InterpolationTableTest extends Command {
    private final MapleInterpolationTable testTable;
    public InterpolationTableTest() {
        try {
            testTable = MapleInterpolationTable.fromConfigFile(MapleConfigFile.fromDeployedConfig("InterpolatedMotorFeedForward", "DrivingMotorOpenLoop"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("InterpolationTables/testValueVelocity", 1.0);
    }

    @Override
    public void execute() {
        assert testTable != null;
        final double vel = SmartDashboard.getNumber("InterpolationTables/testValueVelocity", 1.0);
        SmartDashboard.putNumber(
                "InterpolationTables/testOutPutPower",
                testTable.interpolateVariable("motorPower", vel)
        );
    }
}

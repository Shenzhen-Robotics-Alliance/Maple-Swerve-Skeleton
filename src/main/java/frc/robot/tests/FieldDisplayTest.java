package frc.robot.tests;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.MapleTimeUtils;
import org.littletonrobotics.junction.Logger;

public class FieldDisplayTest extends Command {
    @Override
    public void execute() {
        Logger.recordOutput("Test/Robot", new Pose2d(3, 3, new Rotation2d()));
        Logger.recordOutput("Test/Mechanism", new Pose3d(-0.25, 0.245, 0.09, new Rotation3d(0, -Math.toRadians(15 + 8 + 15 * Math.sin(MapleTimeUtils.getLogTimeSeconds())), 0)));
    }
}
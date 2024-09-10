package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AutoUtils {
    public static Command setIdleForSuperStructureCommand(RobotContainer robot) {
        return Commands.runOnce(() -> setIdleForSuperStructuer(robot), robot.intake, robot.pitch, robot.flyWheels);
    }
    public static void setIdleForSuperStructuer(RobotContainer robot) {
        robot.intake.runIdle();
        robot.flyWheels.runIdle();
        robot.pitch.setIdle();
    }
}

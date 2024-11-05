package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoUtils {
    public static Command getSuperStructureIdleCommand(RobotContainer robot) {
        return robot.flyWheels.getFlyWheelsDefaultCommand().alongWith(robot.pitch.run(robot.pitch::runIdle));
    }
}

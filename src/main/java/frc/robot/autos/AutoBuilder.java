package frc.robot.autos;

import frc.robot.RobotContainer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoBuilder {
    public static LoggedDashboardChooser<Auto> buildAutoChooser(RobotContainer robotContainer) {
        final LoggedDashboardChooser<Auto> autoSendableChooser = new LoggedDashboardChooser<>("Select Auto");
        autoSendableChooser.addDefaultOption("None", Auto.none());
        autoSendableChooser.addOption("Example Auto", new ExampleAuto(robotContainer));

        return autoSendableChooser;
    }
}

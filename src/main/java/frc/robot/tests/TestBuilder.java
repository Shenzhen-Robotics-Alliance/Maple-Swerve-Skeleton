package frc.robot.tests;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

public class TestBuilder {
    public static SendableChooser<Supplier<Command>> buildTestsChooser() {
        final SendableChooser<Supplier<Command>> testsChooser = new SendableChooser<>();
        testsChooser.setDefaultOption("None", Commands::none);
        testsChooser.addOption("Wheels Calibration", WheelsCalibrationCTRE::new);
        testsChooser.addOption("Field Display Test", FieldDisplayTest::new);
        testsChooser.addOption("Robot Simulation Test", PhysicsSimulationTest::new);
        testsChooser.addOption("LED Test", LEDTest::new);
        return testsChooser;
    }
}

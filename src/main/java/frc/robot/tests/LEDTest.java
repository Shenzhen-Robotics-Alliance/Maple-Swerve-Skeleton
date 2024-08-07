package frc.robot.tests;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.utils.LEDAnimation;

public class LEDTest extends SequentialCommandGroup {
    private final LEDStatusLight statusLight;
    public LEDTest() {
        this.statusLight = new LEDStatusLight(0, 155);

        super.addCommands(Commands.runOnce(statusLight::onReset));
        super.addCommands(statusLight.playAnimation(
                new LEDAnimation.Charging(255, 140, 50, 1.5)
        ));

        super.addCommands(statusLight.playAnimation(
                new LEDAnimation.SlideBackAndForth(0,200, 255, 1, 0.8),
                3
        ));

        super.addCommands(statusLight.playAnimation(
                new LEDAnimation.Rainbow(0.6)
        ));
    }
}

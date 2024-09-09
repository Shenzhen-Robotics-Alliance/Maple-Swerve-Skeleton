package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.LEDAnimation;

public class PrepareToAmp extends Command {
    private final Pitch pitch;
    private final FlyWheels flyWheels;
    private final LEDStatusLight statusLight;
    private static final LEDAnimation PREPARING_AMP = new LEDAnimation.Charging(255, 0, 255, 2), // purple charging
            READY_TO_AMP = new LEDAnimation.ShowColor(0, 255, 0); // show color
    public PrepareToAmp(Pitch pitch, FlyWheels flyWheels, LEDStatusLight statusLight) {
        super();
        this.pitch = pitch;
        this.flyWheels = flyWheels;
        this.statusLight = statusLight;
        super.addRequirements(pitch, flyWheels, statusLight);
    }

    private boolean running = false;
    @Override
    public void initialize() {
        running = false;
    }

    @Override
    public void execute() {
        running = true;
        pitch.runSetPointProfiled(Math.toRadians(60));
        flyWheels.runRPMProfiled(1000);

        statusLight.setAnimation(isReady() ? READY_TO_AMP : PREPARING_AMP);
    }

    public boolean isReady() {
        return running && pitch.inPosition() && flyWheels.flyWheelsReady();
    }

    public Command untilReady() {
        return this.until(this::isReady);
    }
}

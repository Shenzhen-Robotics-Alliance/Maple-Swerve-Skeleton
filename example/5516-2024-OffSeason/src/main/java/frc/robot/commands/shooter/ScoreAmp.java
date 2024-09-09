package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.LEDAnimation;
import frc.robot.utils.MapleTimeUtils;

public class ScoreAmp extends Command {
    private final Intake intake;
    private final Pitch pitch;
    private final FlyWheels flyWheels;
    private final LEDStatusLight statusLight;

    private static final LEDAnimation SCORING_AMP = new LEDAnimation.Breathe(0, 255, 0, 2);
    private static final double START_PITCH_DELAY = 0.05;
    public ScoreAmp(Intake intake, Pitch pitch, FlyWheels flyWheels, LEDStatusLight statusLight) {
        super();
        this.intake = intake;
        this.pitch = pitch;
        this.flyWheels = flyWheels;
        this.statusLight = statusLight;
        super.addRequirements(intake, pitch, flyWheels, statusLight);
    }

    private double startTime = 0;
    @Override
    public void initialize() {
        startTime = MapleTimeUtils.getLogTimeSeconds();
    }

    @Override
    public void execute() {
        intake.runFullIntakeVoltage();
        if (startTime > START_PITCH_DELAY)
            pitch.runSetPointProfiled(Math.toRadians(98));
        else
            pitch.runSetPointProfiled(60);
        flyWheels.runRPMProfiled(1000);
        statusLight.setAnimation(SCORING_AMP);
    }

    @Override
    public boolean isFinished() {
        return MapleTimeUtils.getLogTimeSeconds() - startTime > 0.8;
    }
}

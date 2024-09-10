package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.LEDAnimation;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * aims at the speaker continuously
 * adjust the pitch and flywheels to match the current distance
 * it does not adjust the chassis rotation, so it should be running in parallel with {@link frc.robot.commands.drive.JoystickDriveAndAimAtTarget} or {@link frc.robot.commands.drive.AutoAlignment}
 * it takes the chassis velocity into consideration, but will still be inaccurate if moving too fast
 * the command never ends by itself, readyToShoot() tells you whether the pitch and fly-wheels are ready
 * */
public class AimAtSpeakerContinuously extends Command {
    private final FlyWheels flyWheels;
    private final Pitch pitch;
    private final MapleShooterOptimization shooterOptimization;
    private final Supplier<Translation2d> targetPositionSupplier;
    private final HolonomicDriveSubsystem drive;
    private final LEDStatusLight statusLight;
    private final BooleanSupplier additionalCondition;

    private static final LEDAnimation AIMING_SPEAKER = new LEDAnimation.Charging(255, 0, 255, 2),
                AIMING_SPEAKER_READY = new LEDAnimation.ShowColor(0, 255, 0);

    public AimAtSpeakerContinuously(FlyWheels flyWheels, Pitch pitch, LEDStatusLight statusLight, MapleShooterOptimization shooterOptimization, HolonomicDriveSubsystem drive, Supplier<Translation2d> targetPositionSupplier, BooleanSupplier additionalCondition) {
        this.flyWheels = flyWheels;
        this.pitch = pitch;
        this.shooterOptimization = shooterOptimization;
        this.drive = drive;
        this.targetPositionSupplier = targetPositionSupplier;
        this.statusLight = statusLight;
        this.additionalCondition = additionalCondition;

        super.addRequirements(flyWheels, pitch, statusLight);
    }

    boolean shooterOptimizationRunning = false;
    @Override
    public void initialize() {
        shooterOptimizationRunning = false; // this prevents early exit of the command
    }

    @Override
    public void execute() {
        shooterOptimizationRunning = true;
        final MapleShooterOptimization.ShooterState state = shooterOptimization.getOptimizedShootingState(
                targetPositionSupplier.get(),
                drive.getPose().getTranslation(),
                drive.getMeasuredChassisSpeedsFieldRelative()
        );
        state.log("Shooter/");

        pitch.runStaticSetPoint(Math.toRadians(state.shooterAngleDegrees), Math.toRadians(state.shooterAngleChangeRateDegreesPerSecond));
        flyWheels.runStaticRPMSetPoint(state.shooterRPM, state.shooterRPMChangeRateRPMPerSeconds);

        if (statusLight != null)
            statusLight.setAnimation(readyToShoot() ? AIMING_SPEAKER_READY : AIMING_SPEAKER);
    }

    public boolean readyToShoot() {
        return shooterOptimizationRunning
                && shooterOptimization.isTargetInRange(targetPositionSupplier.get(), drive.getPose().getTranslation())
                && additionalCondition.getAsBoolean()
                && flyWheels.flyWheelsReady()
                && pitch.inPosition();
    }

    public Command untilReady() {
        return this.until(this::readyToShoot);
    }
}

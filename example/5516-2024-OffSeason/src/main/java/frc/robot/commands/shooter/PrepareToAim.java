package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.LEDAnimation;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.Supplier;

/**
 * runs a static shooter optimization state calculated at the start of the command
 * on default it does not exit automatically, using this.untilReady() can make the command exit when the shooter is prepared
 * this is used to prepare for shooting
 * */
public class PrepareToAim extends Command {
    private final FlyWheels flyWheels;
    private final Pitch pitch;
    private final LEDStatusLight statusLight;
    private final MapleShooterOptimization shooterOptimization;
    private final Supplier<Translation2d> robotPositionSupplier, targetPositionSupplier;

    private static final LEDAnimation PREPARING_TO_SHOOT = new LEDAnimation.Charging(255, 0, 255, 2);


    /**
     * creates a prepare to aim command
     * the static shooter state is calculated from the robot's estimated pose when the command is initialized
     * */
    public PrepareToAim(FlyWheels flyWheels, Pitch pitch, MapleShooterOptimization shooterOptimization, LEDStatusLight statusLight, HolonomicDriveSubsystem driveSubsystem, Supplier<Translation2d> targetPositionSupplier) {
        this(flyWheels, pitch, shooterOptimization, statusLight, () -> driveSubsystem.getPose().getTranslation(), targetPositionSupplier);
    }

    public PrepareToAim(FlyWheels flyWheels, Pitch pitch, MapleShooterOptimization shooterOptimization, LEDStatusLight statusLight, Supplier<Translation2d> robotPositionSupplier, Supplier<Translation2d> targetPositionSupplier) {
        this.flyWheels = flyWheels;
        this.pitch = pitch;
        this.statusLight = statusLight;
        this.shooterOptimization = shooterOptimization;
        this.robotPositionSupplier = robotPositionSupplier;
        this.targetPositionSupplier = targetPositionSupplier;

        super.addRequirements(flyWheels, pitch, statusLight);
    }

    private MapleShooterOptimization.ShooterState initialState;
    private boolean running = false;
    @Override
    public void initialize() {
        running = false;
        this.initialState = shooterOptimization.getOptimizedShootingState(
                targetPositionSupplier.get(),
                robotPositionSupplier.get(),
                new ChassisSpeeds()
        );
    }


    @Override
    public void execute() {
        running = true;
        flyWheels.runRPMProfiled(initialState.shooterRPM);
        pitch.runSetPointProfiled(Math.toRadians(initialState.shooterAngleDegrees));

        if (statusLight != null)
            statusLight.setAnimation(PREPARING_TO_SHOOT);
    }

    public Command untilReady() {
        return super.beforeStarting(() -> running = false)
                .until(this::isReady);
    }

    public boolean isReady() {
        return running && flyWheels.flyWheelsReady() && pitch.inPosition();
    }
}

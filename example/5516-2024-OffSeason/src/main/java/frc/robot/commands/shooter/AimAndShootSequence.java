package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.CompetitionFieldUtils.CompetitionFieldVisualizer;
import frc.robot.utils.CompetitionFieldUtils.Objects.Crescendo2024FieldObjects;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AimAndShootSequence extends SequentialCommandGroup  {
    public AimAndShootSequence(
            Pitch pitch, FlyWheels flyWheels, Intake intake,
            MapleShooterOptimization shooterOptimization,
            HolonomicDriveSubsystem drive,
            Supplier<Translation2d> targetPositionSupplier,
            BooleanSupplier externalShootCondition,
            LEDStatusLight statusLight,
            CompetitionFieldVisualizer visualizer) {
        this(
                pitch, flyWheels, intake,
                shooterOptimization,
                drive,
                () -> drive.getPose().getTranslation(),
                targetPositionSupplier,
                externalShootCondition,
                statusLight,
                visualizer
        );
    }

    private final Intake intake;
    /**
     * creates a sequence to shoot at speaker in the following steps:
     * 1. runs a {@link PrepareToAim} command, until the shooter is prepared
     * 2. aims at the speaker continuously using a {@link AimAtSpeakerContinuously} command
     * 2. finally, when the shooters are ready and when the external shooting condition is met, execute the shoot
     * */
    public AimAndShootSequence(
            Pitch pitch, FlyWheels flyWheels, Intake intake,
            MapleShooterOptimization shooterOptimization,
            HolonomicDriveSubsystem drive,
            Supplier<Translation2d> robotScoringPositionSupplier,
            Supplier<Translation2d> targetPositionSupplier,
            BooleanSupplier externalShootCondition,
            LEDStatusLight ledStatusLight,
            CompetitionFieldVisualizer visualizer) {

        super();
        this.intake = intake;
        super.addRequirements(pitch, flyWheels, intake);
        super.addCommands(Commands.runOnce(intake::runIdle));

//        super.addCommands(
//                new PrepareToAim(flyWheels, pitch, shooterOptimization, ledStatusLight,  robotScoringPositionSupplier, targetPositionSupplier)
//                        .untilReady()
//        );
        final AimAtSpeakerContinuously aimAtSpeakerContinuously = new AimAtSpeakerContinuously(
                flyWheels, pitch, ledStatusLight, shooterOptimization, drive, targetPositionSupplier, externalShootCondition
        );

        /* visualization */
        final Command visualizeNote;
        if (visualizer == null)
            visualizeNote = Commands.none();
        else visualizeNote = Commands.runOnce(() -> visualizer.addGamePieceOnFly(new Crescendo2024FieldObjects.NoteFlyingToSpeaker(
                new Translation3d(drive.getPose().getX(), drive.getPose().getY(), 0.3),
                shooterOptimization.getFlightTimeSeconds(targetPositionSupplier.get(), robotScoringPositionSupplier.get())
        )));


        final Command visualizeNoteFullCommand = Commands
                .waitUntil(() -> !intake.isNotePresent())
                .andThen(visualizeNote)
                .onlyIf(() -> intake.isNotePresent());
        final Command waitForRightTimingAndShoot = Commands.waitUntil(aimAtSpeakerContinuously::readyToShoot)
                .andThen(intake.executeLaunch().alongWith(visualizeNoteFullCommand));
        super.addCommands(aimAtSpeakerContinuously.raceWith(waitForRightTimingAndShoot));
    }

    public Command ifNotePresent() {
        return this.onlyIf(intake::isNotePresent);
    }
}

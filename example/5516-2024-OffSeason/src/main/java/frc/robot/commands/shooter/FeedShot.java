package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.PitchConstants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.CompetitionFieldUtils.Objects.Crescendo2024FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.Simulations.CompetitionFieldSimulation;

public class FeedShot {
    public static Command prepareToFeedForever(Pitch pitch, FlyWheels flyWheels) {
        final Command preparePitch = Commands.run(() -> pitch.runSetPointProfiled(PitchConstants.PITCH_LOWEST_ROTATION_RAD), pitch),
                prepareFlyWheels = Commands.run(() -> flyWheels.runRPMProfiled(3500), flyWheels);
        return preparePitch.alongWith(prepareFlyWheels);
    }

    public static Command prepareToFeedUntilReady(Pitch pitch, FlyWheels flyWheels) {
        return prepareToFeedForever(pitch, flyWheels).until(
                () -> pitch.inPosition()
                && flyWheels.flyWheelsReady()
        );
    }

    public static Command shootFeed(Pitch pitch, FlyWheels flyWheels, Intake intake, CompetitionFieldSimulation simulation, HolonomicDriveSubsystem driveSubsystem) {
        final SequentialCommandGroup shootFeed = new SequentialCommandGroup();
        final boolean[] noteInShooter = new boolean[] {false};

        shootFeed.addRequirements(pitch, flyWheels, intake);

        shootFeed.addCommands(Commands.runOnce(() -> noteInShooter[0] = intake.isNotePresent()));

        shootFeed.addCommands(prepareToFeedUntilReady(pitch, flyWheels)
                .withTimeout(0.5)
        );

        shootFeed.addCommands(intake.executeLaunch()
                .withTimeout(2)
                .deadlineWith(prepareToFeedForever(pitch, flyWheels))
        );

        /* visualization */
        final Command simulationCommand;
        if (simulation == null) simulationCommand = Commands.none();
        else simulationCommand = Commands.runOnce(() -> simulation.addGamePiece(new Crescendo2024FieldObjects.FeedShotLowNote(
                driveSubsystem.getPose()
                        .getTranslation()
                        .plus(new Translation2d(0.5, 0)
                                .rotateBy(driveSubsystem.getFacing())
                        ),
                driveSubsystem.getFacing()
        )));
        shootFeed.addCommands(simulationCommand.onlyIf(() -> noteInShooter[0]));

        return shootFeed;
    }
}

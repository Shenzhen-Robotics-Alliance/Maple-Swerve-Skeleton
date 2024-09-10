package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.ChassisFaceToRotation;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.CompetitionFieldUtils.CompetitionFieldVisualizer;
import frc.robot.utils.MapleShooterOptimization;

import static frc.robot.constants.FieldConstants.*;

public class AimAtSpeakerFactory {
    public static Command shootAtSpeakerStill(HolonomicDriveSubsystem drive, Intake intake, Pitch pitch, FlyWheels flyWheels, MapleShooterOptimization shooterOptimization, LEDStatusLight statusLight, CompetitionFieldVisualizer visualizer) {
        final Command chassisAimAtSpeaker = ChassisFaceToRotation.faceToTarget(drive, SPEAKER_POSITION_SUPPLIER);

        final Command semiAutoAimAndShoot = new AimAndShootSequence(
                pitch, flyWheels, intake, shooterOptimization, drive,
                FieldConstants.SPEAKER_POSITION_SUPPLIER,
                chassisAimAtSpeaker::isFinished,
                statusLight,
                visualizer
        ).ifNotePresent();

        final Command aimAtSpeakerStill = semiAutoAimAndShoot.deadlineWith(chassisAimAtSpeaker);
        aimAtSpeakerStill.addRequirements(drive, pitch, flyWheels);

        return aimAtSpeakerStill.onlyIf(intake::isNotePresent);
    }
}

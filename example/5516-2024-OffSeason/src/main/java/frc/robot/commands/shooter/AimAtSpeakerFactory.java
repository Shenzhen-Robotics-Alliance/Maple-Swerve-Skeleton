package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.ChassisFaceToRotation;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.MapleShooterOptimization;

import static frc.robot.constants.FieldConstants.*;

public class AimAtSpeakerFactory {
    public static Command shootAtSpeakerStill(HolonomicDriveSubsystem driveSubsystem, Intake intake, Pitch pitch, FlyWheels flyWheels, MapleShooterOptimization shooterOptimization, LEDStatusLight statusLight) {
        final Command chassisAimAtSpeaker = ChassisFaceToRotation.faceToTarget(driveSubsystem, SPEAKER_POSITION_SUPPLIER);
        final AimAtSpeakerContinuously aimAtSpeaker = new AimAtSpeakerContinuously(
                flyWheels, pitch, statusLight, shooterOptimization, driveSubsystem,
                SPEAKER_POSITION_SUPPLIER,
                chassisAimAtSpeaker::isFinished
        );
        final Command waitUntilAimSuccessFullAndShoot = Commands.waitSeconds(0.2)
                .andThen(
                        Commands.waitUntil(() -> chassisAimAtSpeaker.isFinished() && aimAtSpeaker.readyToShoot()))
                .withTimeout(2)
                .andThen(intake.executeLaunch());

        final Command aimAtSpeakerStill = waitUntilAimSuccessFullAndShoot.deadlineWith(chassisAimAtSpeaker.alongWith(aimAtSpeaker));
        aimAtSpeakerStill.addRequirements(driveSubsystem, pitch, flyWheels);
        return aimAtSpeakerStill.onlyIf(intake::isNotePresent);
    }
}

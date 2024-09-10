package frc.robot.commands.shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.CompetitionFieldUtils.CompetitionFieldVisualizer;
import frc.robot.utils.MaplePathPlannerLoader;
import frc.robot.utils.MapleShooterOptimization;

import static frc.robot.constants.FieldConstants.*;

public class FollowPathGrabAndShootStill extends SequentialCommandGroup {
    public FollowPathGrabAndShootStill(PathPlannerPath pathAtBlueAlliance, double distanceToTargetMetersStartPreparing, HolonomicDriveSubsystem driveSubsystem, Intake intake, Pitch pitch, FlyWheels flyWheels, MapleShooterOptimization shooterOptimization, LEDStatusLight statusLight, CompetitionFieldVisualizer visualizer) {
        final Command followPath = AutoBuilder.followPath(pathAtBlueAlliance)
                .andThen(Commands.runOnce(driveSubsystem::stop, driveSubsystem))
                .andThen(Commands.waitUntil(intake::isNotePresent).raceWith(Commands.waitSeconds(1))); // after the robot has stopped, wait for up to 1 sec
        final Command intakeDuringFollowPath = intake.executeIntakeNote();
        final Command prepareToShootDuringFollowPath = Commands.waitUntil(
                () -> MaplePathPlannerLoader.getEndingRobotPoseInCurrentAllianceSupplier(pathAtBlueAlliance).get().getTranslation()
                        .getDistance(driveSubsystem.getPose().getTranslation()) < distanceToTargetMetersStartPreparing
                )
                .andThen(new PrepareToAim(
                        flyWheels, pitch, shooterOptimization, statusLight,
                        () -> MaplePathPlannerLoader.getEndingRobotPoseInCurrentAllianceSupplier(pathAtBlueAlliance).get().getTranslation(),
                        SPEAKER_POSITION_SUPPLIER)
                );

        super.addCommands(followPath.deadlineWith(
                intakeDuringFollowPath.alongWith(prepareToShootDuringFollowPath)
        ));

        super.addCommands(AimAtSpeakerFactory.shootAtSpeakerStill(
                driveSubsystem, intake, pitch, flyWheels, shooterOptimization, statusLight, visualizer
        ));
    }
}

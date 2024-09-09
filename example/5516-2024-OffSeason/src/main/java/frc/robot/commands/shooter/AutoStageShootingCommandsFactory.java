package frc.robot.commands.shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.utils.MaplePathPlannerLoader.*;

public class AutoStageShootingCommandsFactory {
    public static final double PATH_ALMOST_FINISHED_METERS = 0.8,
            PRECISE_ALIGNMENT_SPEED_CONSTRAIN_MPS = 1.5,
            SHOOTING_VELOCITY_LIMIT_MPS = 0.7, SHOOTING_ANGULAR_VELOCITY_LIMIT = Math.toRadians(20);
    public static final Pose2d PRECISE_ALIGNMENT_TOLERANCE = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(3));

    public static Command followPathGrabAndShoot(PathPlannerPath pathAtBlueAlliance, HolonomicDriveSubsystem driveSubsystem, Intake intake, Pitch pitch, FlyWheels flyWheels, MapleShooterOptimization shooterOptimization, LEDStatusLight statusLight, boolean grabNoteAtShootingPosition) {
        final Supplier<Translation2d> scoringPositionSupplier = () -> getEndingRobotPoseInCurrentAllianceSupplier(pathAtBlueAlliance).get().getTranslation();
        final Command followPathUntilAlmostFinished = AutoBuilder.followPath(pathAtBlueAlliance)
                .until(() -> pathAlmostFinish(pathAtBlueAlliance, driveSubsystem.getPose()));
        final Supplier<Pose2d> preciseAlignmentTarget = () -> new Pose2d(
                scoringPositionSupplier.get(),
                shooterOptimization.getShooterFacing(
                        FieldConstants.SPEAKER_POSITION_SUPPLIER.get(),
                        scoringPositionSupplier.get(),
                        new ChassisSpeeds())
        );
        final Command preciseAlignment = new DriveToPose(
                driveSubsystem,
                preciseAlignmentTarget,
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                PRECISE_ALIGNMENT_SPEED_CONSTRAIN_MPS
        );
        final BooleanSupplier robotScoringPoseReached = () -> preciseAlignmentFinished(
                preciseAlignmentTarget.get(),
                driveSubsystem.getPose(),
                driveSubsystem.getMeasuredChassisSpeedsRobotRelative()
        );

        final PrepareToAim prepareToAim = new PrepareToAim(
                flyWheels, pitch, shooterOptimization, statusLight,
                scoringPositionSupplier,
                FieldConstants.SPEAKER_POSITION_SUPPLIER
        );

        final Command intakeNoteDuringBeforeAimSuccess = grabNoteAtShootingPosition ?
                intake.executeIntakeNote().onlyIf(() -> !intake.isNotePresent())
                :Commands.none();
        final Command pushNoteUpwardsAfterAimSuccess = grabNoteAtShootingPosition ?
                Commands.run(intake::runFullIntakeVoltage, intake)
                        .until(intake::isNoteTouchingIntake)
                : Commands.none();
        final Command shootNoteAfterAimSuccess = pushNoteUpwardsAfterAimSuccess
                .andThen(Commands.run(intake::runFullIntakeVoltage, intake).until(() -> !intake.isNotePresent()))
                .withTimeout(2.5);

        final Command overallDriveCommand = followPathUntilAlmostFinished.andThen(preciseAlignment);
        final Command overallIntakeExecution = intakeNoteDuringBeforeAimSuccess
                .andThen(Commands.waitUntil((() -> prepareToAim.isReady() && robotScoringPoseReached.getAsBoolean())))
                .andThen(shootNoteAfterAimSuccess);
        final Command overallShootExecution = overallIntakeExecution.deadlineWith(prepareToAim);
        return overallShootExecution.deadlineWith(overallDriveCommand);
    }

    private static boolean pathAlmostFinish(PathPlannerPath pathAtBlueAlliance, Pose2d currentPose) {
        final Pose2d endingPose = getEndingRobotPoseInCurrentAllianceSupplier(pathAtBlueAlliance).get();
        final double distanceToTarget = currentPose.getTranslation().getDistance(endingPose.getTranslation());
        return  distanceToTarget < PATH_ALMOST_FINISHED_METERS;
    }

    private static boolean preciseAlignmentFinished(Pose2d desiredPose, Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        return Math.abs(desiredPose.getX() - currentPose.getX()) < PRECISE_ALIGNMENT_TOLERANCE.getX()
                &&  Math.abs(desiredPose.getY() - currentPose.getY()) < PRECISE_ALIGNMENT_TOLERANCE.getY()
                && Math.abs(desiredPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees()) < PRECISE_ALIGNMENT_TOLERANCE.getRotation().getDegrees()
                && Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < SHOOTING_VELOCITY_LIMIT_MPS
                && Math.abs(currentSpeeds.omegaRadiansPerSecond) < SHOOTING_ANGULAR_VELOCITY_LIMIT;
    }
}

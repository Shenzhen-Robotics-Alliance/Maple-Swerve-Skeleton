package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.CompetitionFieldUtils.CompetitionFieldVisualizer;
import frc.robot.utils.CompetitionFieldUtils.Objects.Crescendo2024FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.Objects.GamePieceOnFlyDisplay;
import frc.robot.utils.CompetitionFieldUtils.Simulations.CompetitionFieldSimulation;
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
    private boolean hasNote = false;
    @Override
    public void initialize() {
        startTime = MapleTimeUtils.getLogTimeSeconds();
        hasNote = intake.isNotePresent();
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

    // Visualization
    public Command withVisualization(CompetitionFieldVisualizer visualizer, CompetitionFieldSimulation simulation, HolonomicDriveSubsystem driveSubsystem) {
        final Command visualizeAmpNote = Commands.runOnce(() -> {
            final Transform2d poseError = FieldConstants
                    .toCurrentAlliancePose(CORRECT_SCORE_AMP_ROBOT_POSE_BLUE)
                    .minus(driveSubsystem.getPose());
            final Translation2d shooterPositionOnField = driveSubsystem.getPose()
                    .getTranslation()
                    .plus(new Translation2d(-0.3, 0).rotateBy(driveSubsystem.getFacing()));
            if (poseError.getTranslation().getNorm() < 0.25
                    && Math.abs(poseError.getRotation().getDegrees()) < 8)
                visualizer.addGamePieceOnFly(new AmpSuccessNote(shooterPositionOnField));
            else
                visualizer.addGamePieceOnFly(new AmpFailedNote(
                        driveSubsystem.getPose().getRotation(),
                        shooterPositionOnField,
                        simulation
                ));
        });
        final Command visualizationCommand = Commands
                .waitSeconds(0.3)
                .andThen(visualizeAmpNote)
                .onlyIf(() -> hasNote && visualizer != null);
        return this.alongWith(visualizationCommand);
    }

    private static final double
            SHOOTER_HEIGHT_AMP = 0.4,
            AMP_INITIAL_VELOCITY = 3.5,
            AMP_FLIGHT_TIME_SECONDS = 0.8;
    private static final Translation3d AMP_POSITION = new Translation3d(1.81, 8.35, 0);
    private static final Pose2d CORRECT_SCORE_AMP_ROBOT_POSE_BLUE = new Pose2d(1.81, 7.71, Rotation2d.fromDegrees(-90));
    private static final class AmpSuccessNote extends GamePieceOnFlyDisplay {
        public AmpSuccessNote(Translation2d robotPosition) {
            super(
                    new Translation3d(robotPosition.getX(), robotPosition.getY(), SHOOTER_HEIGHT_AMP),
                    FieldConstants.toCurrentAllianceTranslation(AMP_POSITION),
                    AMP_FLIGHT_TIME_SECONDS
            );
        }

        @Override
        public String getTypeName() {
            return "Note";
        }

        @Override
        public Pose3d getPose3d() {
            /*
            * the amplified note follows the free-fall law
            * h = h0 + v*t - 1/2*t^2*g;
            *  */
            final double t = getTimeSinceLaunchSeconds(),
                    height = SHOOTER_HEIGHT_AMP + AMP_INITIAL_VELOCITY * t
                            - 1.0/2.0 * t * t * 10;
            return new Pose3d(
                    new Translation3d(super.getPose3d().getX(), super.getPose3d().getY(), height),
                    new Rotation3d(Math.toRadians(90), 0, 0)
            );
        }
    }

    private static final class AmpFailedNote extends GamePieceOnFlyDisplay {
        private final Rotation2d robotFacingWhenLaunching;
        public AmpFailedNote(Rotation2d robotFacingWhenLaunching, Translation2d shooterPositionOnField, CompetitionFieldSimulation simulation) {
            super(
                    new Translation3d(shooterPositionOnField.getX(), shooterPositionOnField.getY(), SHOOTER_HEIGHT_AMP),
                    getNoteTouchGroundTranslation3d(
                            shooterPositionOnField.plus(
                                    new Translation2d(-0.7, 0).rotateBy(robotFacingWhenLaunching)
                            )),
                    AMP_FLIGHT_TIME_SECONDS
            );
            this.robotFacingWhenLaunching = robotFacingWhenLaunching;
            if (simulation != null) CommandScheduler.getInstance().schedule(
                    Commands.waitSeconds(AMP_FLIGHT_TIME_SECONDS).andThen(Commands.runOnce(
                            () -> simulation.addGamePiece(new Crescendo2024FieldObjects.NoteOnFieldSimulated(
                                    super.targetedPosition.toTranslation2d()
                            ))
                    ))
            );
        }

        private static Translation3d getNoteTouchGroundTranslation3d(Translation2d noteEndingTranslation2d) {
            return new Translation3d(
                    noteEndingTranslation2d.getX(),
                    noteEndingTranslation2d.getY(),
                    0.2
            );
        }

        @Override
        public String getTypeName() {
            return "Note";
        }

        @Override
        public Pose3d getPose3d() {
            final double t = getTimeSinceLaunchSeconds(),
                    height = SHOOTER_HEIGHT_AMP + AMP_INITIAL_VELOCITY * t
                            - 1.0/2.0 * t * t * 10;
            return new Pose3d(
                    new Translation3d(super.getPose3d().getX(), super.getPose3d().getY(), height),
                    new Rotation3d(0, Math.toRadians(90), robotFacingWhenLaunching.getRadians())
            );
        }
    }
}

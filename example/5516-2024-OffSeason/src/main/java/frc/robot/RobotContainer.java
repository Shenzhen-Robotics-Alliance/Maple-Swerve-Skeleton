// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.AmpSideSixNotesFast;
import frc.robot.autos.Auto;
import frc.robot.autos.PathPlannerAuto;
import frc.robot.commands.drive.*;
import frc.robot.commands.shooter.*;
import frc.robot.constants.*;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.IO.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.subsystems.vision.apriltags.AprilTagVisionIOReal;
import frc.robot.subsystems.vision.apriltags.ApriltagVisionIOSim;
import frc.robot.utils.CompetitionFieldUtils.CompetitionFieldVisualizer;
import frc.robot.utils.CompetitionFieldUtils.Objects.Crescendo2024FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.Simulations.CompetitionFieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulations.Crescendo2024FieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulations.OpponentRobotSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulations.SwerveDriveSimulation;
import frc.robot.subsystems.vision.apriltags.PhotonCameraProperties;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.MapleShooterOptimization;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // pdp for akit logging
    public final PowerDistribution powerDistribution;
    // Subsystems
    public final SwerveDrive drive;
    public final AprilTagVision aprilTagVision;
    public final LEDStatusLight ledStatusLight;
    public final Intake intake;
    public final Pitch pitch;
    public final FlyWheels flyWheels;

    /* shoot commands */
    public final MapleShooterOptimization shooterOptimization;

    // Controller
    private final CommandXboxController driverXBox = new CommandXboxController(0);


    public enum JoystickMode {
        LEFT_HANDED,
        RIGHT_HANDED
    }
    // Dashboard Selections
    private final LoggedDashboardChooser<JoystickMode> driverModeChooser;
    private final LoggedDashboardChooser<Auto> autoChooser;
    private final SendableChooser<Supplier<Command>> testChooser;

    // Simulation and Field Visualization
    private final CompetitionFieldVisualizer competitionFieldVisualizer;
    public final CompetitionFieldVisualizer visualizerForShooter;
    private CompetitionFieldSimulation fieldSimulation;

    private final LoggedDashboardChooser<Double> shootingDistanceChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        final List<PhotonCameraProperties> camerasProperties =
                // PhotonCameraProperties.loadCamerasPropertiesFromConfig("5516-2024-OffSeason-Vision"); // loads camera properties from deploy/PhotonCamerasProperties/5516-2024-OffSeason-Vision.xml
                VisionConstants.photonVisionCameras; // load configs stored directly in VisionConstants.java

        this.ledStatusLight = new LEDStatusLight(0, 155);
        final BooleanConsumer noteInShooterConsumer = ledStatusLight::setNotePresent;

        switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                powerDistribution = new PowerDistribution(0, PowerDistribution.ModuleType.kCTRE);

                /* CTRE Chassis: */
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.CTRE_ON_CANIVORE,
                        new GyroIOPigeon2(TunerConstants.DrivetrainConstants),
                        new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, "FrontLeft"),
                        new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.FrontRight, "FrontRight"),
                        new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.BackLeft, "BackLeft"),
                        new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.BackRight, "BackRight")
                );

                aprilTagVision = new AprilTagVision(
                        new AprilTagVisionIOReal(camerasProperties),
                        camerasProperties,
                        drive
                );

                this.competitionFieldVisualizer = new CompetitionFieldVisualizer(drive::getPose);

                this.intake = new Intake(
                        new IntakeIOReal(16, 2, 1),
                        noteInShooterConsumer
                );
                this.pitch = new Pitch(new PitchIOReal(
                        17, true,
                        18, false
                ));
                this.flyWheels = new FlyWheels(new FlyWheelIO[]{
                        new FlyWheelIOReal(21, true),
                        new FlyWheelIOReal(22, true)
                });
            }

            case SIM -> {
                powerDistribution = new PowerDistribution();
                // Sim robot, instantiate physics sim IO implementations
                final ModuleIOSim
                        frontLeft = new ModuleIOSim(),
                        frontRight = new ModuleIOSim(),
                        backLeft = new ModuleIOSim(),
                        backRight = new ModuleIOSim();
                final GyroIOSim gyroIOSim = new GyroIOSim();
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.GENERIC,
                        gyroIOSim,
                        frontLeft, frontRight, backLeft, backRight
                );
                final SwerveDriveSimulation driveSimulation = new SwerveDriveSimulation(
                        gyroIOSim,
                        frontLeft, frontRight, backLeft, backRight,
                        new Pose2d(3, 3, new Rotation2d()),
                        drive::setPose
                );
                fieldSimulation = new Crescendo2024FieldSimulation(driveSimulation);
                this.competitionFieldVisualizer = fieldSimulation.getVisualizer();

                aprilTagVision = new AprilTagVision(
                        new ApriltagVisionIOSim(
                                camerasProperties,
                                VisionConstants.fieldLayout,
                                driveSimulation::getObjectOnFieldPose2d
                        ),
                        camerasProperties,
                        drive
                );

                fieldSimulation.placeGamePiecesOnField();

                fieldSimulation.addRobot(new OpponentRobotSimulation(0));
                fieldSimulation.addRobot(new OpponentRobotSimulation(1));
                fieldSimulation.addRobot(new OpponentRobotSimulation(2));

                final IntakeIOSim intakeIOSim = new IntakeIOSim();
                fieldSimulation.registerIntake(intakeIOSim);
                this.intake = new Intake(intakeIOSim, noteInShooterConsumer);
                this.pitch = new Pitch(new PitchIOSim());
                this.flyWheels = new FlyWheels(new FlyWheelIO[]{
                        new FlyWheelIOSim(intakeIOSim),
                        new FlyWheelIOSim(intakeIOSim)
                });
            }

            default -> {
                powerDistribution = new PowerDistribution();
                // Replayed robot, disable IO implementations
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.GENERIC,
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {}
                );

                aprilTagVision = new AprilTagVision(
                        (inputs) -> {},
                        camerasProperties,
                        drive
                );

                this.competitionFieldVisualizer = new CompetitionFieldVisualizer(drive::getPose);

                this.intake = new Intake((inputs) -> {}, noteInShooterConsumer);
                this.pitch = new Pitch((inputs) -> {});
                this.flyWheels = new FlyWheels(new FlyWheelIO[]{
                        (inputs) -> {},
                        (inputs) -> {}
                });
            }
        }

        this.drive.configHolonomicPathPlannerAutoBuilder(competitionFieldVisualizer);
        visualizerForShooter = Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ?
                null
                : competitionFieldVisualizer;

        SmartDashboard.putData("Select Test", testChooser = buildTestsChooser());
        autoChooser = buildAutoChooser();

        driverModeChooser = new LoggedDashboardChooser<>("Driver Mode", new SendableChooser<>());
        driverModeChooser.addDefaultOption(JoystickMode.LEFT_HANDED.name(), JoystickMode.LEFT_HANDED);
        driverModeChooser.addOption(JoystickMode.RIGHT_HANDED.name(), JoystickMode.RIGHT_HANDED);

        /* you can tune the numbers on dashboard and copy-paste them to here */
        this.shooterOptimization = new MapleShooterOptimization(
                "MainShooter",
                new double[] {1.4, 2, 3, 3.5, 4, 4.5, 4.8},
                new double[] {54, 47, 35, 33, 30.5, 25, 25},
                new double[] {3000, 3000, 3500, 3700, 4000, 4300, 4500},
                new double[] {0.1, 0.1, 0.1, 0.12, 0.12, 0.15, 0.15}
        );

        this.shootingDistanceChooser = new LoggedDashboardChooser<>("Select Shooting Distance");
        this.shootingDistanceChooser.addDefaultOption("1.4", 1.4);
        this.shootingDistanceChooser.addOption("2", 2.0);
        this.shootingDistanceChooser.addOption("3", 3.0);
        this.shootingDistanceChooser.addOption("3.5", 3.5);
        this.shootingDistanceChooser.addOption("4", 4.0);

        configureButtonBindings();
        configureAutoNamedCommands();
    }

    private void configureAutoNamedCommands() {
        // TODO: bind your named commands during auto here
        NamedCommands.registerCommand("my named command", Commands.runOnce(
                () -> System.out.println("my named command executing!!!")
        ));
    }

    private static LoggedDashboardChooser<Auto> buildAutoChooser() {
        final LoggedDashboardChooser<Auto> autoSendableChooser = new LoggedDashboardChooser<>("Select Auto");
        autoSendableChooser.addDefaultOption("None", Auto.none());
        autoSendableChooser.addOption("Amp Side Six Notes Fast", new AmpSideSixNotesFast());
        autoSendableChooser.addOption("Example Pathplanner Auto", new PathPlannerAuto("Example Auto", new Pose2d(1.3, 7.2, new Rotation2d())));
        SmartDashboard.putData("Select Auto", autoSendableChooser.getSendableChooser());

        // TODO: add your autos here
        return autoSendableChooser;
    }

    private static SendableChooser<Supplier<Command>> buildTestsChooser() {
        final SendableChooser<Supplier<Command>> testsChooser = new SendableChooser<>();
        testsChooser.setDefaultOption("None", Commands::none);
        // TODO add your tests here (system identification and etc.)
        return testsChooser;
    }

    private boolean isDSPresentedAsRed = FieldConstants.isSidePresentedAsRed();
    private boolean isLeftHanded = true;
    private Command autonomousCommand = Commands.none();
    private Auto previouslySelectedAuto = null;
    /**
     * reconfigures button bindings if alliance station has changed
     * re-create autos if not yet created
     * */
    public void checkForCommandChanges() {
        final boolean isLeftHandedSelected = !JoystickMode.RIGHT_HANDED.equals(driverModeChooser.get());
        if (FieldConstants.isSidePresentedAsRed() != isDSPresentedAsRed || isLeftHanded != isLeftHandedSelected)
            configureButtonBindings();
        isDSPresentedAsRed = FieldConstants.isSidePresentedAsRed();
        isLeftHanded = isLeftHandedSelected;

        final Auto selectedAuto = autoChooser.get();
        if (selectedAuto != previouslySelectedAuto) {
            this.autonomousCommand = selectedAuto
                    .getAutoCommand(this)
                    .beforeStarting(() -> resetFieldAndOdometryForAuto(selectedAuto.getStartingPoseAtBlueAlliance()))
                    .finallyDo(MapleSubsystem::disableAllSubsystems);
        }
        previouslySelectedAuto = selectedAuto;
    }

    private void resetFieldAndOdometryForAuto(Pose2d robotStartingPoseAtBlueAlliance) {
        final Pose2d startingPose = FieldConstants.toCurrentAlliancePose(robotStartingPoseAtBlueAlliance);
        drive.setPose(startingPose);

        if (fieldSimulation == null) return;
        fieldSimulation.getMainRobot().setSimulationWorldPose(startingPose);
        fieldSimulation.resetFieldForAuto();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */
    public void configureButtonBindings() {
        /* joystick drive command */
        final MapleJoystickDriveInput driveInput = JoystickMode.RIGHT_HANDED.equals(driverModeChooser.get()) ?
                MapleJoystickDriveInput.rightHandedJoystick(driverXBox)
                : MapleJoystickDriveInput.leftHandedJoystick(driverXBox);
        final JoystickDrive joystickDrive = new JoystickDrive(
                driveInput,
                () -> true,
                driverXBox.getHID()::getPOV,
                drive
        );
        drive.setDefaultCommand(joystickDrive);

        /* lock chassis with x-formation */
        driverXBox.x().whileTrue(Commands.run(drive::lockChassisWithXFormation, drive));

        /* reset gyro heading manually (in case the vision does not work) */
        driverXBox.start().onTrue(Commands.runOnce(
                        () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                        drive
                ).ignoringDisable(true)
        );

        /* intake commands */
        driverXBox.leftTrigger(0.5).whileTrue(intake.executeIntakeNote(
                joystickDrive,
                ledStatusLight,
                driverXBox.getHID()
        ));
        /* split note from bottom */
        driverXBox.a().whileTrue(Commands.run(
                () -> {
                    intake.runInvertVoltage();
                    flyWheels.forceMaxRevert();
                }
        ));

        /* amp command */
        driverXBox.y()
                .onTrue(new PrepareToAmp(pitch, flyWheels, ledStatusLight))
                .whileTrue(Commands.run(() -> joystickDrive.setCurrentRotationalMaintenance(
                        FieldConstants.toCurrentAllianceRotation(Rotation2d.fromDegrees(-90))
                )))
                .onFalse(new ScoreAmp(intake, pitch, flyWheels, ledStatusLight));

        /* feed shot */
        driverXBox.b()
                .onTrue(FeedShot.prepareToFeedForever(pitch, flyWheels))
                .whileTrue(Commands.run(() -> joystickDrive.setCurrentRotationalMaintenance(
                        FieldConstants.toCurrentAllianceRotation(Rotation2d.fromDegrees(180))
                )))
                .onFalse(FeedShot.shootFeed(pitch, flyWheels, intake));

        /* aim and shoot command */
        final JoystickDriveAndAimAtTarget faceTargetWhileDrivingLowSpeed = new JoystickDriveAndAimAtTarget(
                driveInput, drive,
                FieldConstants.SPEAKER_POSITION_SUPPLIER,
                shooterOptimization,
                0.5
        );
        final Command semiAutoAimAndShoot = new AimAndShootSequence(
                pitch, flyWheels, intake, shooterOptimization, drive,
                FieldConstants.SPEAKER_POSITION_SUPPLIER,
                faceTargetWhileDrivingLowSpeed::chassisRotationInPosition,
                ledStatusLight,
                visualizerForShooter
        );
        driverXBox.rightTrigger(0.5).whileTrue(semiAutoAimAndShoot.deadlineWith(faceTargetWhileDrivingLowSpeed));

        driverXBox.rightBumper().whileTrue(new PathFindToPoseAndShootSequence(
                intake, pitch, flyWheels, shooterOptimization, drive,
                () -> FieldConstants.toCurrentAllianceTranslation(new Translation2d(shootingDistanceChooser.get() + 0.1, 5.55)),
                () -> FieldConstants.toCurrentAllianceTranslation(new Translation2d(shootingDistanceChooser.get(), 5.55)),
                FieldConstants.SPEAKER_POSITION_SUPPLIER,
                ledStatusLight,
                visualizerForShooter
        ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonomousCommand;
    }

    public Command getTestCommand() {
        return testChooser.getSelected().get();
    }

    public void updateFieldSimAndDisplay() {
        if (fieldSimulation != null)
            fieldSimulation.updateSimulationWorld();

        competitionFieldVisualizer.updateObjectsToDashboardAndTelemetry();

        ShooterVisualizer.showResultsToDashboard(competitionFieldVisualizer.mainRobot.getPose3d());
    }
}

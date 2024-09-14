// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.utils.CompetitionFieldUtils.Simulations.CompetitionFieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulations.Crescendo2024FieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulations.OpponentRobotSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulations.SwerveDriveSimulation;
import frc.robot.subsystems.vision.apriltags.PhotonCameraProperties;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.MapleShooterOptimization;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.ArrayList;
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
    private CompetitionFieldSimulation fieldSimulation = null;
    private List<OpponentRobotSimulation> opponentRobotSimulations = new ArrayList<>();

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

                this.competitionFieldVisualizer = new CompetitionFieldVisualizer(drive);

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

                opponentRobotSimulations.add(new OpponentRobotSimulation(0, fieldSimulation));
                fieldSimulation.addRobot(opponentRobotSimulations.get(0));
                opponentRobotSimulations.add(new OpponentRobotSimulation(1, fieldSimulation));
                fieldSimulation.addRobot(opponentRobotSimulations.get(1));
                opponentRobotSimulations.add(new OpponentRobotSimulation(2, fieldSimulation));
                fieldSimulation.addRobot(opponentRobotSimulations.get(2));
                opponentRobotSimulations.add(new OpponentRobotSimulation(3, fieldSimulation));
                fieldSimulation.addRobot(opponentRobotSimulations.get(3));
                opponentRobotSimulations.add(new OpponentRobotSimulation(4, fieldSimulation));
                fieldSimulation.addRobot(opponentRobotSimulations.get(4));

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

                this.competitionFieldVisualizer = new CompetitionFieldVisualizer(drive);

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

        /* you can tune these shooter params on the dashboard, changes will apply real-time */
        this.shooterOptimization = new MapleShooterOptimization(
                "MainShooter",
                /* shooting distances */
                new double[] {1.4, 2, 3, 3.5, 4, 4.5, 5, 5.5},
                /* corresponding shooter angles (degrees) */
                new double[] {54, 47, 35, 33, 30.5, 25, 25, 25},
                /* corresponding flywheels RPM */
                new double[] {3000, 3000, 3500, 3700, 4000, 4300, 4500, 4700},
                /*
                * flight-time in seconds
                * this affects how much system will "think ahead"
                * it also affects the animation of the flying note
                * to measure this number precisely, record a 120fps video with your phone of your robot shooting
                * */
                new double[] {0.22, 0.25, 0.28, 0.3, 0.34, 0.36, 0.4, 0.42}
        );

        this.shootingDistanceChooser = new LoggedDashboardChooser<>("Select Shooting Distance");
        this.shootingDistanceChooser.addDefaultOption("4", 4.0);
        this.shootingDistanceChooser.addOption("3.5", 3.5);
        this.shootingDistanceChooser.addOption("3", 3.0);
        this.shootingDistanceChooser.addOption("2", 2.0);
        this.shootingDistanceChooser.addOption("1.5", 2.0);

        configureButtonBindings();
        configureAutoNamedCommands();
    }

    public void teleOpInit() {
        for (OpponentRobotSimulation opponentRobotSimulation: opponentRobotSimulations)
            opponentRobotSimulation.teleOpInit();
    }

    private void configureAutoNamedCommands() {
        /*
        * our auto is built from custom auto builder
        * so there is no named commands
        *  */
    }

    private static LoggedDashboardChooser<Auto> buildAutoChooser() {
        final LoggedDashboardChooser<Auto> autoSendableChooser = new LoggedDashboardChooser<>("Select Auto");
        autoSendableChooser.addDefaultOption("None", Auto.none());
        autoSendableChooser.addOption("Amp Side Six Notes Fast", new AmpSideSixNotesFast());
        autoSendableChooser.addOption("Example Pathplanner Auto", new PathPlannerAuto("Example Auto", new Pose2d(1.3, 7.2, new Rotation2d())));
        SmartDashboard.putData("Select Auto", autoSendableChooser.getSendableChooser());
        return autoSendableChooser;
    }

    private SendableChooser<Supplier<Command>> buildTestsChooser() {
        final SendableChooser<Supplier<Command>> testsChooser = new SendableChooser<>();
        testsChooser.setDefaultOption("None", Commands::none);

        /* flywheels system identifications */
        /* select the test in SmartDashBoard/SelectTest and enable the robot in test mode to start them */
        testsChooser.addOption(
                "FlyWheel1 SysId Dynamic (Forward)",
                () -> flyWheels.sysIdRoutines[1].dynamic(SysIdRoutine.Direction.kForward)
        );
        testsChooser.addOption(
                "FlyWheel1 SysId Dynamic (Reverse)",
                () -> flyWheels.sysIdRoutines[1].dynamic(SysIdRoutine.Direction.kReverse)
        );
        testsChooser.addOption(
                "FlyWheel1 SysId Quasi Static (Forward)",
                () -> flyWheels.sysIdRoutines[1].quasistatic(SysIdRoutine.Direction.kForward)
        );
        testsChooser.addOption(
                "FlyWheel1 SysId Quasi Static (Reverse)",
                () -> flyWheels.sysIdRoutines[1].quasistatic(SysIdRoutine.Direction.kReverse)
        );
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
        isLeftHanded = isLeftHandedSelected;

        final Auto selectedAuto = autoChooser.get();
        if (FieldConstants.isSidePresentedAsRed() != isDSPresentedAsRed || selectedAuto != previouslySelectedAuto) {
            this.autonomousCommand = selectedAuto
                    .getAutoCommand(this)
                    .finallyDo(MapleSubsystem::disableAllSubsystems);
            resetFieldAndOdometryForAuto(selectedAuto.getStartingPoseAtBlueAlliance());
        }

        previouslySelectedAuto = selectedAuto;
        isDSPresentedAsRed = FieldConstants.isSidePresentedAsRed();
    }

    private void resetFieldAndOdometryForAuto(Pose2d robotStartingPoseAtBlueAlliance) {
        final Pose2d startingPose = FieldConstants.toCurrentAlliancePose(robotStartingPoseAtBlueAlliance);

        if (fieldSimulation != null) {
            fieldSimulation.getMainRobot().setSimulationWorldPose(startingPose);
            fieldSimulation.resetFieldForAuto();
            updateFieldSimAndDisplay();
        }

        drive.periodic();
        drive.setPose(startingPose);
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

        /* drive slowly and intake note, rumble game-pad when note detected */
        driverXBox
                .leftTrigger(0.5)
                .whileTrue(intake.executeIntakeNote(ledStatusLight, driverXBox.getHID()))
                .onTrue(Commands.runOnce(() -> joystickDrive.setSensitivity(0.6, 0.4)))
                .onFalse(Commands.runOnce(joystickDrive::resetSensitivity));
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
                .whileTrue(Commands.run(() -> {
                    joystickDrive.setCurrentRotationalMaintenance(
                        FieldConstants.toCurrentAllianceRotation(Rotation2d.fromDegrees(-90)));
                    joystickDrive.setSensitivity(0.7, 1);
                }).finallyDo(joystickDrive::resetSensitivity))
                .onFalse(new ScoreAmp(intake, pitch, flyWheels, ledStatusLight)
                        .withVisualization(visualizerForShooter, fieldSimulation, drive)
                );

        /* feed shot */
        driverXBox.b()
                .onTrue(FeedShot.prepareToFeedForever(pitch, flyWheels))
                .whileTrue(Commands.run(() -> joystickDrive.setCurrentRotationalMaintenance(
                        FieldConstants.toCurrentAllianceRotation(Rotation2d.fromDegrees(180))
                )))
                .onFalse(FeedShot.shootFeed(pitch, flyWheels, intake, fieldSimulation, drive));

        /* aim and shoot command */
        final JoystickDriveAndAimAtTarget faceTargetWhileDrivingLowSpeed = new JoystickDriveAndAimAtTarget(
                driveInput, drive,
                FieldConstants.SPEAKER_POSITION_SUPPLIER,
                shooterOptimization,
                0.3
        );
        final Command semiAutoAimAndShoot = new AimAndShootSequence(
                pitch, flyWheels, intake, shooterOptimization, drive,
                FieldConstants.SPEAKER_POSITION_SUPPLIER,
                faceTargetWhileDrivingLowSpeed::chassisRotationInPosition,
                ledStatusLight,
                visualizerForShooter
        ).ifNotePresent();
        driverXBox.rightTrigger(0.5).whileTrue(semiAutoAimAndShoot
                        .deadlineWith(faceTargetWhileDrivingLowSpeed)
                        .finallyDo(() -> joystickDrive.setCurrentRotationalMaintenance(drive.getFacing()))
        );

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

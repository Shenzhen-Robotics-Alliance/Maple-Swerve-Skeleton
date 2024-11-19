// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.drive.*;
import frc.robot.constants.*;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.IO.*;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.subsystems.vision.apriltags.AprilTagVisionIOReal;
import frc.robot.subsystems.vision.apriltags.ApriltagVisionIOSim;
import frc.robot.subsystems.vision.apriltags.PhotonCameraProperties;
import frc.robot.utils.AIRobotInSimulation;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.MapleShooterOptimization;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // pdp for akit logging
    public final PowerDistribution powerDistribution;
    // Subsystems
    public final SwerveDrive drive;
    public final AprilTagVision aprilTagVision;

    /* an example shooter optimization */
    public final MapleShooterOptimization exampleShooterOptimization;

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

    // Simulated drive
    private final SwerveDriveSimulation driveSimulation;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        final List<PhotonCameraProperties> camerasProperties =
                // PhotonCameraProperties.loadCamerasPropertiesFromConfig("5516-2024-OffSeason-Vision"); //
                // loads camera properties from
                // deploy/PhotonCamerasProperties/5516-2024-OffSeason-Vision.xml
                VisionConstants.photonVisionCameras; // load configs stored directly in VisionConstants.java

        switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                driveSimulation = null;

                powerDistribution = new PowerDistribution(0, PowerDistribution.ModuleType.kCTRE);

                /* CTRE Chassis: */
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.CTRE_ON_CANIVORE,
                        new GyroIOPigeon2(TunerConstants.DrivetrainConstants),
                        new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, "FrontLeft"),
                        new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.FrontRight, "FrontRight"),
                        new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.BackLeft, "BackLeft"),
                        new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.BackRight, "BackRight"));

                /* REV Chassis */
                //                drive = new SwerveDrive(
                //                        SwerveDrive.DriveType.CTRE_ON_CANIVORE,
                //                        new GyroIOPigeon2(TunerConstants.DrivetrainConstants),
                //                        new ModuleIOSpark(0),
                //                        new ModuleIOSpark(1),
                //                        new ModuleIOSpark(2),
                //                        new ModuleIOSpark(3)
                //                );

                aprilTagVision =
                        new AprilTagVision(new AprilTagVisionIOReal(camerasProperties), camerasProperties, drive);
            }

            case SIM -> {
                this.driveSimulation = new SwerveDriveSimulation(
                        DriveTrainSimulationConfig.Default()
                                .withRobotMass(DriveTrainConstants.ROBOT_MASS)
                                .withBumperSize(DriveTrainConstants.BUMPER_LENGTH, DriveTrainConstants.BUMPER_WIDTH)
                                .withTrackLengthTrackWidth(
                                        DriveTrainConstants.TRACK_LENGTH, DriveTrainConstants.TRACK_WIDTH)
                                .withSwerveModule(() -> new SwerveModuleSimulation(
                                        DriveTrainConstants.DRIVE_MOTOR,
                                        DriveTrainConstants.STEER_MOTOR,
                                        DriveTrainConstants.DRIVE_GEAR_RATIO,
                                        DriveTrainConstants.STEER_GEAR_RATIO,
                                        DriveTrainConstants.DRIVE_CURRENT_LIMIT,
                                        DriveTrainConstants.STEER_CURRENT_LIMIT,
                                        DriveTrainConstants.DRIVE_FRICTION_VOLTAGE,
                                        DriveTrainConstants.STEER_FRICTION_VOLTAGE,
                                        DriveTrainConstants.WHEEL_RADIUS,
                                        DriveTrainConstants.STEER_INERTIA,
                                        DriveTrainConstants.WHEEL_COEFFICIENT_OF_FRICTION))
                                .withGyro(DriveTrainConstants.gyroSimulationFactory),
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                powerDistribution = new PowerDistribution();
                // Sim robot, instantiate physics sim IO implementations
                final ModuleIOSim frontLeft = new ModuleIOSim(driveSimulation.getModules()[0]),
                        frontRight = new ModuleIOSim(driveSimulation.getModules()[1]),
                        backLeft = new ModuleIOSim(driveSimulation.getModules()[2]),
                        backRight = new ModuleIOSim(driveSimulation.getModules()[3]);
                final GyroIOSim gyroIOSim = new GyroIOSim(driveSimulation.getGyroSimulation());
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.GENERIC, gyroIOSim, frontLeft, frontRight, backLeft, backRight);

                aprilTagVision = new AprilTagVision(
                        new ApriltagVisionIOSim(
                                camerasProperties,
                                VisionConstants.fieldLayout,
                                driveSimulation::getSimulatedDriveTrainPose),
                        camerasProperties,
                        drive);

                SimulatedArena.getInstance().resetFieldForAuto();
                AIRobotInSimulation.startOpponentRobotSimulations();
            }

            default -> {
                this.driveSimulation = null;

                powerDistribution = new PowerDistribution();
                // Replayed robot, disable IO implementations
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.GENERIC,
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {});

                aprilTagVision = new AprilTagVision((inputs) -> {}, camerasProperties, drive);
            }
        }

        this.drive.configHolonomicPathPlannerAutoBuilder();

        SmartDashboard.putData("Select Test", testChooser = buildTestsChooser());
        autoChooser = buildAutoChooser();

        driverModeChooser = new LoggedDashboardChooser<>("Driver Mode", new SendableChooser<>());
        driverModeChooser.addDefaultOption(JoystickMode.LEFT_HANDED.name(), JoystickMode.LEFT_HANDED);
        driverModeChooser.addOption(JoystickMode.RIGHT_HANDED.name(), JoystickMode.RIGHT_HANDED);

        /* you can tune the numbers on dashboard and copy-paste them to here */
        this.exampleShooterOptimization = new MapleShooterOptimization(
                "ExampleShooter",
                new double[] {1.4, 2, 3, 3.5, 4, 4.5, 4.8},
                new double[] {54, 49, 37, 33.5, 30.5, 25, 25},
                new double[] {3000, 3000, 3500, 3700, 4000, 4300, 4500},
                new double[] {0.1, 0.1, 0.1, 0.12, 0.12, 0.15, 0.15});

        configureButtonBindings();
        configureAutoNamedCommands();
    }

    private void configureAutoNamedCommands() {
        // TODO: bind your named commands during auto here
        NamedCommands.registerCommand(
                "my named command", Commands.runOnce(() -> System.out.println("my named command executing!!!")));
    }

    private void configureAutoTriggers(PathPlannerAuto pathPlannerAuto) {

        pathPlannerAuto.event("hello world").onTrue(Commands.runOnce(() -> System.out.println("hello world!!!")));
    }

    private LoggedDashboardChooser<Auto> buildAutoChooser() {
        final LoggedDashboardChooser<Auto> autoSendableChooser = new LoggedDashboardChooser<>("Select Auto");
        autoSendableChooser.addDefaultOption("None", Auto.none());
        autoSendableChooser.addOption(
                "Example Custom Auto With PathPlanner Trajectories",
                new ExampleCustomAutoWithPathPlannerTrajectories());
        autoSendableChooser.addOption(
                "Example Custom Auto With Choreo Trajectories: Rush", new ExampleCustomAutoWithChoreoTrajectories());
        autoSendableChooser.addOption(
                "Example Custom Auto With Choreo Trajectories", new ExampleCustomAutoWithChoreoTrajectories2());
        autoSendableChooser.addOption("Example Pathplanner Auto", new PathPlannerAutoWrapper("Example Auto"));
        autoSendableChooser.addOption("Example Face To Target", new ExampleFaceToTarget());
        // TODO: add your autos here

        SmartDashboard.putData("Select Auto", autoSendableChooser.getSendableChooser());
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
    /** reconfigures button bindings if alliance station has changed re-create autos if not yet created */
    public void checkForCommandChanges() {
        final boolean isLeftHandedSelected = !JoystickMode.RIGHT_HANDED.equals(driverModeChooser.get());
        if (FieldConstants.isSidePresentedAsRed() != isDSPresentedAsRed || isLeftHanded != isLeftHandedSelected)
            configureButtonBindings();
        isLeftHanded = isLeftHandedSelected;

        final Auto selectedAuto = autoChooser.get();
        if (FieldConstants.isSidePresentedAsRed() != isDSPresentedAsRed || selectedAuto != previouslySelectedAuto) {
            try {
                this.autonomousCommand =
                        selectedAuto.getAutoCommand(this).finallyDo(MapleSubsystem::disableAllSubsystems);
                configureAutoTriggers(
                        new PathPlannerAuto(autonomousCommand, selectedAuto.getStartingPoseAtBlueAlliance()));
            } catch (Exception e) {
                this.autonomousCommand = Commands.none();
                DriverStation.reportError(
                        "Error Occurred while obtaining autonomous command: \n"
                                + e.getMessage()
                                + "\n"
                                + Arrays.toString(e.getStackTrace()),
                        false);
                throw new RuntimeException(e);
            }
            resetFieldAndOdometryForAuto(selectedAuto.getStartingPoseAtBlueAlliance());
        }

        previouslySelectedAuto = selectedAuto;
        isDSPresentedAsRed = FieldConstants.isSidePresentedAsRed();
    }

    private void resetFieldAndOdometryForAuto(Pose2d robotStartingPoseAtBlueAlliance) {
        final Pose2d startingPose = FieldConstants.toCurrentAlliancePose(robotStartingPoseAtBlueAlliance);

        if (driveSimulation != null) {
            driveSimulation.setSimulationWorldPose(startingPose);
            SimulatedArena.getInstance().resetFieldForAuto();
            updateFieldSimAndDisplay();
        }

        drive.periodic();
        drive.setPose(startingPose);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link XboxController}), and then passing it to
     * a {@link JoystickButton}.
     */
    public void configureButtonBindings() {
        /* joystick drive command */
        final MapleJoystickDriveInput driveInput = JoystickMode.RIGHT_HANDED.equals(driverModeChooser.get())
                ? MapleJoystickDriveInput.rightHandedJoystick(driverXBox)
                : MapleJoystickDriveInput.leftHandedJoystick(driverXBox);
        final JoystickDrive joystickDrive =
                new JoystickDrive(driveInput, () -> true, driverXBox.getHID()::getPOV, drive);
        drive.setDefaultCommand(joystickDrive);

        /* lock chassis with x-formation */
        driverXBox.x().whileTrue(Commands.run(drive::lockChassisWithXFormation, drive));

        /* reset gyro heading manually (in case the vision does not work) */
        driverXBox
                .start()
                .onTrue(Commands.runOnce(
                                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                                drive)
                        .ignoringDisable(true));

        /* TODO: aim at target and drive example, delete it for your project */
        final JoystickDriveAndAimAtTarget exampleFaceTargetWhileDriving = new JoystickDriveAndAimAtTarget(
                driveInput, drive, FieldConstants.SPEAKER_POSITION_SUPPLIER, exampleShooterOptimization, 0.75);
        driverXBox.rightTrigger(0.5).whileTrue(exampleFaceTargetWhileDriving);

        /* auto alignment example, delete it for your project */
        final AutoAlignment exampleAutoAlignment = new AutoAlignment(
                drive,
                /* (position of AMP) */
                () -> FieldConstants.toCurrentAlliancePose(new Pose2d(1.85, 7.3, Rotation2d.fromDegrees(90))),
                () -> FieldConstants.toCurrentAlliancePose(new Pose2d(1.85, 7.74, Rotation2d.fromDegrees(90))),
                new Pose2d(0.04, 0.04, Rotation2d.fromDegrees(2)),
                0.8,
                2);
        driverXBox.b().whileTrue(exampleAutoAlignment);
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
        if (driveSimulation == null) return;
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Notes",
                SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(Pose3d[]::new));
        Logger.recordOutput("FieldSimulation/OpponentRobotPositions", AIRobotInSimulation.getOpponentRobotPoses());
        Logger.recordOutput(
                "FieldSimulation/AlliancePartnerRobotPositions", AIRobotInSimulation.getAlliancePartnerRobotPoses());
    }
}

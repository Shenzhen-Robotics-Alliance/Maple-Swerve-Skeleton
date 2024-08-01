// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.AutoAlignment;
import frc.robot.commands.drive.CustomFollowPath;
import frc.robot.commands.drive.CustomFollowPathOnFly;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.IO.GyroIOPigeon2;
import frc.robot.subsystems.drive.IO.GyroIOSim;
import frc.robot.subsystems.drive.IO.ModuleIOSim;
import frc.robot.subsystems.drive.IO.ModuleIOTalonFX;
import frc.robot.tests.*;
import frc.robot.utils.CompetitionFieldUtils.Simulation.Crescendo2024FieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.OpponentRobotSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.SwerveDriveSimulation;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.MaplePathPlannerLoader;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.io.IOException;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final PowerDistribution powerDistribution;
    // Subsystems
    private final SwerveDrive drive;

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0),
            operatorController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardChooser<Supplier<Command>> testChooser;

    // Simulation
    private Crescendo2024FieldSimulation fieldSimulation = null;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        final MapleConfigFile chassisCalibrationFile;
        try {
            chassisCalibrationFile = MapleConfigFile.fromDeployedConfig(
                    "ChassisWheelsCalibration",
                    Constants.chassisConfigName
            );
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        final MapleConfigFile.ConfigBlock generalConfigBlock = chassisCalibrationFile.getBlock("GeneralInformation");
        switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
//                drive = new SwerveDrive(
//                        new GyroIOPigeon2(),
//                        new ModuleIOSparkMax(0),
//                        new ModuleIOSparkMax(1),
//                        new ModuleIOSparkMax(2),
//                        new ModuleIOSparkMax(3)
//                );
                 drive = new SwerveDrive(
                         new GyroIOPigeon2(),
                         new ModuleIOTalonFX(chassisCalibrationFile.getBlock("FrontLeft"), generalConfigBlock),
                         new ModuleIOTalonFX(chassisCalibrationFile.getBlock("FrontRight"), generalConfigBlock),
                         new ModuleIOTalonFX(chassisCalibrationFile.getBlock("BackLeft"), generalConfigBlock),
                         new ModuleIOTalonFX(chassisCalibrationFile.getBlock("BackRight"), generalConfigBlock),
                         generalConfigBlock
                 );
            }

            case SIM -> {
                powerDistribution = new PowerDistribution();
                // Sim robot, instantiate physics sim IO implementations
                final ModuleIOSim
                        frontLeft = new ModuleIOSim(generalConfigBlock),
                        frontRight = new ModuleIOSim(generalConfigBlock),
                        backLeft = new ModuleIOSim(generalConfigBlock),
                        backRight = new ModuleIOSim(generalConfigBlock);
                final GyroIOSim gyroIOSim = new GyroIOSim();
                drive = new SwerveDrive(
                        gyroIOSim,
                        frontLeft, frontRight, backLeft, backRight,
                        generalConfigBlock
                );
                fieldSimulation = new Crescendo2024FieldSimulation(new SwerveDriveSimulation(
                        generalConfigBlock,
                        gyroIOSim,
                        frontLeft, frontRight, backLeft, backRight,
                        drive.kinematics,
                        new Pose2d(3, 3, new Rotation2d()),
                        drive::setPose
                ));
                fieldSimulation.placeGamePiecesOnField();

                fieldSimulation.addRobot(new OpponentRobotSimulation(0));
                fieldSimulation.addRobot(new OpponentRobotSimulation(1));
                fieldSimulation.addRobot(new OpponentRobotSimulation(2));
            }

            default -> {
                powerDistribution = new PowerDistribution();
                // Replayed robot, disable IO implementations
                drive = new SwerveDrive(
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        generalConfigBlock
                );
            }
        }

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        testChooser = new LoggedDashboardChooser<>("Test Choices", new SendableChooser<>());

        addTestsToChooser();
        configureButtonBindings();
    }

    public void addTestsToChooser() {
        testChooser.addDefaultOption("None", Commands::none);
        testChooser.addOption("Wheels Calibration", WheelsCalibrationCTRE::new);
        testChooser.addOption("Field Display Test", FieldDisplayTest::new);
        testChooser.addOption("Robot Simulation Test", PhysicsSimulationTest::new);
    }

    private boolean isDSPresentedAsRed = Constants.isSidePresentedAsRed();
    /**
     * reconfigures button bindings if alliance station has changed
     * */
    public void checkForAllianceStationChange() {
        if (Constants.isSidePresentedAsRed() != isDSPresentedAsRed)
            configureButtonBindings();
        isDSPresentedAsRed = Constants.isSidePresentedAsRed();
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drive.setDefaultCommand(new JoystickDrive(
                MapleJoystickDriveInput.leftHandedJoystick(driverController),
                () -> true,
                drive
        ));
        driverController.x().whileTrue(Commands.run(drive::lockChassisWithXFormation, drive));
        driverController.b().onTrue(Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive
                ).ignoringDisable(true)
        );

        driverController.y().whileTrue(new CustomFollowPath(
                drive,
                MaplePathPlannerLoader.fromPathFileReversed(
                        "Test Path",
                        new PathConstraints(3, 6, 6.28, 10),
                        new GoalEndState(0, new Rotation2d())
                ).flipPath(),
                "Drive/TestPath"
        ).rePlannedOnStart());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }


    public Command getTestCommand() {
      return testChooser.get().get();
    }

    public void updateSimulationWorld() {
        if (fieldSimulation != null)
            fieldSimulation.updateSimulationWorld();
    }
}

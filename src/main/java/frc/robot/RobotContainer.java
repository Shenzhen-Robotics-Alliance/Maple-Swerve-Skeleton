// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.IO.GyroIOPigeon2;
import frc.robot.subsystems.drive.IO.ModuleIOSim;
import frc.robot.subsystems.drive.IO.ModuleIOTalonFX;
import frc.robot.tests.FieldDisplayTest;
import frc.robot.tests.InterpolationTableTest;
import frc.robot.tests.UnitTest;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.MapleJoystickDriveInput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.io.IOException;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final SwerveDrive drive;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(String chassisName) {
        final MapleConfigFile chassisCalibrationFile;
        try {
            chassisCalibrationFile = MapleConfigFile.fromDeployedConfig("ChassisWheelsCalibration", chassisName);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        final MapleConfigFile.ConfigBlock generalConfigBlock = chassisCalibrationFile.getBlock("GeneralInformation");
        switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
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
                // Sim robot, instantiate physics sim IO implementations
                drive = new SwerveDrive(
                        (inputs) -> {},
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        generalConfigBlock
                );
            }

            default -> {
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

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drive.setDefaultCommand(drive.joystickDrive(
                MapleJoystickDriveInput.leftHandedJoystick(controller),
                true
        ));
        controller.x().whileTrue(Commands.run(drive::lockChassisWithXFormation, drive));
        controller.b().onTrue(Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive
                ).ignoringDisable(true)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }


    public UnitTest getUnitTest() {
        return new FieldDisplayTest();
    }
}

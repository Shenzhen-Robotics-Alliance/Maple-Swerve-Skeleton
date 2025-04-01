// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ReefConstants;
import frc.robot.constants.RobotMode;
import frc.robot.constants.VisionConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    public enum RobotName {
        // Team 5516 dev bot
        TEAM_5516_DEVBOT_HYDROXIDE_I,
        // Team 5516 comp bot
        TEAM_5516_COMPBOT_HYDROXIDE_II,
        // Team 6706 comp bot
        TEAM_6706_COMPBOT
    }

    public static final double defaultPeriodSecs = 0.02;
    public static final boolean LOG_DETAILS = true; // isSimulation();
    private static final RobotMode JAVA_SIM_MODE = RobotMode.SIM;
    public static final RobotMode CURRENT_ROBOT_MODE = isReal() ? RobotMode.REAL : JAVA_SIM_MODE;
    public static final RobotName CURRENT_ROBOT = RobotName.TEAM_5516_COMPBOT_HYDROXIDE_II;
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }
            case SIM -> {
                // Running a physics simulator
                // Log to CodeDirectory/logs if you want to test logging system in a simulation
                // Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(
                        LogFileUtil.addPathSuffix(logPath, "_replayed"),
                        WPILOGWriter.AdvantageScopeOpenBehavior.ALWAYS));
            }
        }

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        // Start AdvantageKit logger
        Logger.start();

        // Performance Optimization
        ReefConstants.loadStatic();
        SignalLogger.enableAutoLogging(false);
        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
        AprilTagFieldLayout.loadField(VisionConstants.CURRENT_FIELD);
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.updateTelemetryAndLED();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        robotContainer.checkForCommandChanges();
    }

    @Override
    public void disabledExit() {
        robotContainer.setMotorBrake(true);
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {}

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
    private Command testCommand = Commands.none();

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().schedule(testCommand = robotContainer.getTestCommand());
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
        testCommand.cancel();
        robotContainer.configureButtonBindings();
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        robotContainer.updateFieldSimAndDisplay();
    }
}

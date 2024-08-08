// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.MapleSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private static final Constants.RobotMode JAVA_SIM_MODE = Constants.RobotMode.SIM;
    public static final Constants.RobotMode CURRENT_ROBOT_MODE = isReal() ? Constants.RobotMode.REAL : JAVA_SIM_MODE;
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
            case REAL, SIM -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                // Running a physics simulator, log to $PROJECT_FOLDER/logs
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replayed")));
            }
        }

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        // Start AdvantageKit logger
        Logger.start();

        MapleSubsystem.subsystemsInit();
    }

    /**
     * This function is called periodically during all modes.
     */
    @Override
    public void robotPeriodic() {
        if (CURRENT_ROBOT_MODE == Constants.RobotMode.SIM)
            robotContainer.updateSimulationWorld();
        MapleSubsystem.checkForOnDisableAndEnable();
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
        robotContainer.rebindKeysIfChanged();
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null)
            autonomousCommand.schedule();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        robotContainer.rebindKeysIfChanged();
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().schedule(robotContainer.getTestCommand());
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
        robotContainer.configureButtonBindings();
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}

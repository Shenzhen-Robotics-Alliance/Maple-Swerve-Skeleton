package frc.robot.utils.CompetitionFieldUtils.Simulation;

import frc.robot.utils.Config.MapleConfigFile;

/**
 * simulates the behavior of our robot
 * it has all the physics behavior as a simulated holonomic chassis
 * in addition to that, it simulates the swerve module behaviors
 * the class is like the bridge between ModuleIOSim and HolonomicChassisSimulation
 * it reads the motor power from ModuleIOSim
 * and feed the result of the physics simulation back to ModuleIOSim, to simulate the odometry encoders' readings
 * TODO write this class
 * */
public class SwerveDriveSimulation extends HolonomicChassisSimulation {
    public SwerveDriveSimulation(MapleConfigFile.ConfigBlock chassisGeneralInfoBlock) {
        super(new RobotProfile(chassisGeneralInfoBlock));
    }
}

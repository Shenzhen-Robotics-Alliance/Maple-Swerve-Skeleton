// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import frc.robot.subsystems.drive.OdometryThread;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public enum RobotMode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static final class ChassisConfigs {
        public enum ChassisType {
            REV,
            CTRE_ON_RIO,
            CTRE_ON_CANIVORE
        }
        public static final ChassisType chassisType = ChassisType.CTRE_ON_CANIVORE;

        public static final String DEFAULT_CHASSIS_CANIVORE = "ChassisCanivore";

        public static final int ODOMETRY_CACHE_CAPACITY = 20;
        public static final double ODOMETRY_FREQUENCY = 250;
        public static final double ODOMETRY_WAIT_TIMEOUT_SECONDS = 0.02;
    }
}

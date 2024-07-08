// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.MechanismControl.MapleSimplePIDController;

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

    public static final class LogConfigs {
        // avoid typos
        public static final String
                SENSORS_INPUTS_PATH = "RawInputs/",
                SENSORS_PROCESSED_INPUTS_PATH = "ProcessedInputs/",
                SYSTEM_PERFORMANCE_PATH = "SystemPerformance/";
    }

    public static final class DriveConfigs {
        public static final double nonUsageTimeResetWheels = 1;

        public static final double deadBandWhenOtherAxisEmpty = 0.02;
        public static final double deadBandWhenOtherAxisFull = 0.1;
        public static final double linearSpeedInputExponent = 1.6;
        public static final double rotationSpeedInputExponent = 2;
    }

    public static final class SwerveDriveConfigs {
        public enum SwerveDriveType {
            REV,
            CTRE_ON_RIO,
            CTRE_ON_CANIVORE
        }
        public static final SwerveDriveType SWERVE_DRIVE_TYPE = SwerveDriveType.CTRE_ON_CANIVORE;

        public static final String CHASSIS_CANBUS = "ChassisCanivore";

        public static final int ODOMETRY_CACHE_CAPACITY = 10;
        public static final double ODOMETRY_FREQUENCY = 250;
        public static final double ODOMETRY_WAIT_TIMEOUT_SECONDS = 0.02;
    }

    public static final class ChassisDefaultConfigs {
        public static final int DEFAULT_GYRO_PORT = 0;
        public static final double DEFAULT_GEAR_RATIO = 6.12;
        public static final double DEFAULT_WHEEL_RADIUS_METERS = 0.051; // 2 inch
        public static final double DEFAULT_BUMPER_WIDTH_METERS = 0.876; // 34.5 inch
        public static final double DEFAULT_HORIZONTAL_WHEELS_MARGIN_METERS = 0.53;
        public static final double DEFAULT_VERTICAL_WHEELS_MARGIN_METERS = 0.53;
        public static final double DEFAULT_BUMPER_LENGTH_METERS = 0.876; // 34.5 inch
        public static final double DEFAULT_MAX_VELOCITY_METERS_PER_SECOND = 4.172; // calculated from Choreo (Kraken x60 motor, 6.12 gear ratio, 55kg robot mass)
        public static final double DEFAULT_MAX_ACCELERATION_METERS_PER_SQUARED_SECOND = 10.184; // calculated from Choreo (Kraken x60 motor, 6.12 gear ratio, 55kg robot mass)
        public static final double DEFAULT_MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND = 540;
    }

    public static final class WheelCalibrationConfigs {
        public static final class WheelToBeCalibrated {
            public final String name;
            public final int drivingMotorID, steeringMotorID, encoderID, drivingMotorPortOnPDP, steeringMotorPortOnPDP;
            public boolean steeringMotorInverted;

            public WheelToBeCalibrated(String name, int drivingMotorID, int steeringMotorID, int encoderID) {
                this(name, drivingMotorID, steeringMotorID, encoderID, -1, -1);
            }

            private WheelToBeCalibrated(String name, int drivingMotorID, int steeringMotorID, int encoderID, int drivingMotorPortOnPDP, int steeringMotorPortOnPDP) {
                this.name = name;
                this.drivingMotorID = drivingMotorID;
                this.steeringMotorID = steeringMotorID;
                this.encoderID = encoderID;
                this.drivingMotorPortOnPDP = drivingMotorPortOnPDP;
                this.steeringMotorPortOnPDP = steeringMotorPortOnPDP;
                this.steeringMotorInverted = false;
            }
        }
        public static final WheelToBeCalibrated[] wheelsToBeCalibrated = new WheelToBeCalibrated[] {
                new WheelToBeCalibrated("FrontLeft", 3, 4, 10),
                new WheelToBeCalibrated("FrontRight", 6, 5, 11),
                new WheelToBeCalibrated("BackLeft", 1, 2, 9),
                new WheelToBeCalibrated("BackRight", 8, 7, 12)
        };
    }

    public static final class SwerveModuleConfigs {
        public static final double MINIMUM_USAGE_SPEED_METERS_PER_SECOND = ChassisDefaultConfigs.DEFAULT_MAX_VELOCITY_METERS_PER_SECOND * 0.03;
        public static final double NON_USAGE_TIME_RESET_SWERVE = 0.5;

        public static final MapleSimplePIDController.SimplePIDProfile steerHeadingCloseLoopConfig = new MapleSimplePIDController.SimplePIDProfile(
                0.9,
                Math.toRadians(65),
                0.02,
                Math.toRadians(1.5),
                0.05,
                true
        );
        public static final double STEERING_CURRENT_LIMIT = 20;
        public static final double DRIVING_CURRENT_LIMIT = 60;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    }
}

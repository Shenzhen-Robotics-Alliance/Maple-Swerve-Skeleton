package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * stores the constants and PID configs for chassis
 * because we want an all-real simulation for the chassis, the numbers are required to be considerably precise
 * */
public class DriveTrainConstants {
    /**
     * numbers that needs to be changed to fit each robot
     * TODO: change these numbers to match your robot
     *  */
    public static final double
            WHEEL_COEFFICIENT_OF_FRICTION = 1.05, // yeah, I know this is against the law of physics, but it feels realistic
            ROBOT_MASS_KG = 40; // robot weight with bumpers

    /**
     *  TODO: change motor type to match your robot
     *  */
    public static final DCMotor
            DRIVE_MOTOR = DCMotor.getKrakenX60(1),
            STEER_MOTOR = DCMotor.getFalcon500(1);

    /**
     * numbers imported from {@link TunerConstants}
     * TODO: for REV chassis, replace them with actual numbers
     * */
    public static final double
            WHEEL_RADIUS_METERS = Units.inchesToMeters(TunerConstants.kWheelRadiusInches),
            DRIVE_GEAR_RATIO = TunerConstants.kDriveGearRatio,
            STEER_GEAR_RATIO = TunerConstants.kSteerGearRatio,
            TIME_ROBOT_STOP_ROTATING_SECONDS = 0.06,
            STEER_FRICTION_VOLTAGE = TunerConstants.kSteerFrictionVoltage,
            DRIVE_FRICTION_VOLTAGE = TunerConstants.kDriveFrictionVoltage,
            DRIVE_INERTIA = 0.01,
            STEER_INERTIA = 0.01;

    /* adjust current limit */
    public static final double DRIVE_CURRENT_LIMIT = 60;
    public static final double STEER_CURRENT_LIMIT = 20;


    /**
     * translations of the modules to the robot center, in FL, FR, BL, BR
     * */
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
            new Translation2d(
                    Units.inchesToMeters(TunerConstants.kFrontLeftXPosInches),
                    Units.inchesToMeters(TunerConstants.kFrontLeftYPosInches)
            ),
            new Translation2d(
                    Units.inchesToMeters(TunerConstants.kFrontRightXPosInches),
                    Units.inchesToMeters(TunerConstants.kFrontRightYPosInches)
            ),
            new Translation2d(
                    Units.inchesToMeters(TunerConstants.kBackLeftXPosInches),
                    Units.inchesToMeters(TunerConstants.kBackLeftYPosInches)
            ),
            new Translation2d(
                    Units.inchesToMeters(TunerConstants.kBackRightXPosInches),
                    Units.inchesToMeters(TunerConstants.kBackRightYPosInches)
            )
    };

    /* equations that calculates some constants for the simulator (don't modify) */
    private static final double GRAVITY_CONSTANT = 9.8;
    public static final double
            DRIVE_BASE_RADIUS = MODULE_TRANSLATIONS[0].getNorm(),
            /* friction_force = normal_force * coefficient_of_friction */
            MAX_FRICTION_ACCELERATION = GRAVITY_CONSTANT * WHEEL_COEFFICIENT_OF_FRICTION,
            MAX_FRICTION_FORCE_PER_MODULE = MAX_FRICTION_ACCELERATION * ROBOT_MASS_KG / MODULE_TRANSLATIONS.length,
            /* force = torque / distance */
            MAX_PROPELLING_FORCE_NEWTONS = DRIVE_MOTOR.getTorque(DRIVE_CURRENT_LIMIT) * DRIVE_GEAR_RATIO / WHEEL_RADIUS_METERS,
            /* floor_speed = wheel_angular_velocity * wheel_radius */
            CHASSIS_MAX_VELOCITY = DRIVE_MOTOR.freeSpeedRadPerSec / DRIVE_GEAR_RATIO * WHEEL_RADIUS_METERS , // calculate using choreo
            CHASSIS_MAX_ACCELERATION_MPS_SQ = Math.min(
                    MAX_FRICTION_ACCELERATION, // cannot exceed max friction
                    MAX_PROPELLING_FORCE_NEWTONS / ROBOT_MASS_KG
            ),
            CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = CHASSIS_MAX_VELOCITY / DRIVE_BASE_RADIUS,
            CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = CHASSIS_MAX_ACCELERATION_MPS_SQ / DRIVE_BASE_RADIUS,
            CHASSIS_FRICTIONAL_ANGULAR_ACCELERATION = CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC / TIME_ROBOT_STOP_ROTATING_SECONDS;

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    /* for collision detection in simulation */
    public static final double
            BUMPER_WIDTH_METERS = Units.inchesToMeters(34.5),
            BUMPER_LENGTH_METERS = Units.inchesToMeters(36),
            /* https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction */
            BUMPER_COEFFICIENT_OF_FRICTION = 0.65,
            /* https://simple.wikipedia.org/wiki/Coefficient_of_restitution */
            BUMPER_COEFFICIENT_OF_RESTITUTION = 0.08;

    /* Gyro Sim */
    public static final double GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ = 100;
    public static final double SKIDDING_AMOUNT_AT_THRESHOLD_RAD = Math.toRadians(1.2);
    /*
     * https://store.ctr-electronics.com/pigeon-2/
     * for a well-installed one with vibration reduction, only 0.4 degree
     * but most teams just install it directly on the rigid chassis frame (including my team :D)
     * so at least 1.2 degrees of drifting in 1 minutes for an average angular velocity of 60 degrees/second
     * which is the average velocity during normal swerve-circular-offense
     * */
    public static final double NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD = Math.toRadians(1.2);
    public static final double AVERAGE_VELOCITY_RAD_PER_SEC_DURING_TEST = Math.toRadians(60);

    /* dead configs, don't change them */
    public static final int ODOMETRY_CACHE_CAPACITY = 10;
    public static final double ODOMETRY_FREQUENCY = 250;
    public static final double ODOMETRY_WAIT_TIMEOUT_SECONDS = 0.02;
    public static final int SIMULATION_TICKS_IN_1_PERIOD = 5;
}

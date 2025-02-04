package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * stores the constants and PID configs for chassis because we want an all-real simulation for the chassis, the numbers
 * are required to be considerably precise
 */
public class DriveTrainConstants {
    /** numbers that needs to be changed to fit each robot TODO: change these numbers to match your robot */
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.5;

    public static final Mass ROBOT_MASS = Kilograms.of(50); // robot weight with bumpers

    /** TODO: change motor type to match your robot */
    public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1);

    public static final DCMotor STEER_MOTOR = DCMotor.getFalcon500(1);

    /** numbers imported from {@link TunerConstants} TODO: for REV chassis, replace them with actual numbers */
    public static final Distance WHEEL_RADIUS = Meters.of(TunerConstants.FrontLeft.WheelRadius);

    public static final double DRIVE_GEAR_RATIO = TunerConstants.FrontLeft.DriveMotorGearRatio;
    public static final double STEER_GEAR_RATIO = TunerConstants.FrontLeft.SteerMotorGearRatio;

    public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage);
    public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage);
    public static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.025);

    /* adjust current limit */
    public static final Current DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT = Amps.of(100);
    public static final Current DRIVE_OVER_CURRENT_PROTECTION = Amps.of(120);
    public static final Time DRIVE_OVERHEAT_PROTECTION_TIME = Seconds.of(1);
    public static final Current DRIVE_OVERHEAT_PROTECTION = Amps.of(80);
    public static final Current STEER_CURRENT_LIMIT = Amps.of(20);

    /** translations of the modules to the robot center, in FL, FR, BL, BR */
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };

    public static final Distance TRACK_LENGTH =
            MODULE_TRANSLATIONS[0].minus(MODULE_TRANSLATIONS[3]).getMeasureX();
    public static final Distance TRACK_WIDTH =
            MODULE_TRANSLATIONS[0].minus(MODULE_TRANSLATIONS[3]).getMeasureY();

    /* equations that calculates some constants for the simulator (don't modify) */
    private static final double GRAVITY_CONSTANT = 9.8;

    public static final Distance DRIVE_BASE_RADIUS = Meters.of(MODULE_TRANSLATIONS[0].getNorm());

    /* friction_force = normal_force * coefficient_of_friction */
    public static final LinearAcceleration MAX_FRICTION_ACCELERATION =
            MetersPerSecondPerSecond.of(GRAVITY_CONSTANT * WHEEL_COEFFICIENT_OF_FRICTION);

    /* force = torque / distance */
    public static final Force MAX_PROPELLING_FORCE = NewtonMeters.of(
                    DRIVE_MOTOR.getTorque(DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT.in(Amps)) * DRIVE_GEAR_RATIO)
            .div(WHEEL_RADIUS)
            .times(4);

    /* floor_speed = wheel_angular_velocity * wheel_radius */
    public static final LinearVelocity CHASSIS_MAX_VELOCITY = MetersPerSecond.of(DRIVE_MOTOR.getSpeed(
                    DRIVE_MOTOR.getTorque(DRIVE_MOTOR.getCurrent(0, TunerConstants.FrontLeft.DriveFrictionVoltage)), 12)
            / DRIVE_GEAR_RATIO
            * WHEEL_RADIUS.in(Meters));
    public static final LinearAcceleration CHASSIS_MAX_ACCELERATION =
            (LinearAcceleration) Measure.min(MAX_FRICTION_ACCELERATION, MAX_PROPELLING_FORCE.div(ROBOT_MASS));
    public static final AngularVelocity CHASSIS_MAX_ANGULAR_VELOCITY =
            RadiansPerSecond.of(CHASSIS_MAX_VELOCITY.in(MetersPerSecond) / DRIVE_BASE_RADIUS.in(Meters));
    public static final AngularAcceleration CHASSIS_MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(
            CHASSIS_MAX_ACCELERATION.in(MetersPerSecondPerSecond) / DRIVE_BASE_RADIUS.in(Meters) * 2);

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    /* for collision detection in simulation */
    public static final Distance BUMPER_WIDTH = Inches.of(30), BUMPER_LENGTH = Inches.of(30);

    // https://unacademy.com/content/upsc/study-material/physics/moment-of-inertia-of-rectangle-section/
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(ROBOT_MASS.in(Kilograms)
            * (BUMPER_WIDTH.in(Meters) * BUMPER_WIDTH.in(Meters) + BUMPER_LENGTH.in(Meters) * BUMPER_LENGTH.in(Meters))
            / 12.0);

    public static final Supplier<GyroSimulation> gyroSimulationFactory = COTS.ofPigeon2();

    /* dead configs, don't change them */
    public static final int ODOMETRY_CACHE_CAPACITY = 10;
    public static final double ODOMETRY_FREQUENCY = 250;
    public static final double ODOMETRY_WAIT_TIMEOUT_SECONDS = 0.02;
    public static final int SIMULATION_TICKS_IN_1_PERIOD = 5;
}

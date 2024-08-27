// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.CustomPIDs.MaplePIDController;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;
import java.util.function.Supplier;

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

    public static final String ROBOT_NAME = "5516-2024-OffSeason";

    // avoid typo errors
    public static final class LogConfigs {
        public static final String
                SYSTEM_PERFORMANCE_PATH = "SystemPerformance/",
                PHYSICS_SIMULATION_PATH = "MaplePhysicsSimulation/",
                APRIL_TAGS_VISION_PATH = "Vision/AprilTags/";
    }

    public static final class CrescendoField2024Constants {
        public static final double FIELD_WIDTH = 16.54;
        public static final double FIELD_HEIGHT = 8.21;

        public static final Translation3d SPEAKER_POSE_BLUE = new Translation3d(0.1, 5.55, 2.2);

        public static final Supplier<Translation2d> SPEAKER_POSITION_SUPPLIER = () -> toCurrentAllianceTranslation(SPEAKER_POSE_BLUE.toTranslation2d());
    }

    public static final class DriverJoystickConfigs {
        public static final double nonUsageTimeResetWheels = 1;

        public static final double deadBandWhenOtherAxisEmpty = 0.02;
        public static final double deadBandWhenOtherAxisFull = 0.1;
        public static final double linearSpeedInputExponent = 1.6;
        public static final double rotationSpeedInputExponent = 2;

        /** the amount of time that the chassis needs to accelerate to the maximum linear velocity */
        public static final double linearAccelerationSmoothOutSeconds = 0.1;
        /** the amount of time that the chassis needs to accelerate to the maximum angular velocity */
        public static final double angularAccelerationSmoothOutSeconds = 0.1;

        public static final double timeActivateRotationMaintenanceAfterNoRotationalInputSeconds = 0.3;
    }

    public static final class SwerveDriveChassisConfigs {
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

        public static final MaplePIDController.MaplePIDConfig chassisRotationalPIDConfig = new MaplePIDController.MaplePIDConfig(
                Math.toRadians(360),
                Math.toRadians(60),
                0.02,
                Math.toRadians(3),
                0.05,
                true,
                0
        );
        public static final TrapezoidProfile.Constraints chassisRotationalConstraints = new TrapezoidProfile.Constraints(Math.toRadians(540), Math.toRadians(540) / 0.6);

        public static final MaplePIDController.MaplePIDConfig chassisTranslationPIDConfig = new MaplePIDController.MaplePIDConfig(
                3,
                0.6,
                0.01,
                0.03,
                0.3,
                false,
                0
        );

        public static final MaplePIDController.MaplePIDConfig chassisTranslationPIDConfigPathFollowing = new MaplePIDController.MaplePIDConfig(
                2,
                1.2,
                0,
                0.03,
                0,
                false,
                0
        );
    }

    public static final class SwerveModuleConfigs {
        public static final MaplePIDController.MaplePIDConfig steerHeadingCloseLoopConfig = new MaplePIDController.MaplePIDConfig(
                0.5,
                Math.toRadians(90),
                0.02,
                Math.toRadians(1.5),
                0,
                true,
                0
        );
        public static final double STEERING_CURRENT_LIMIT = 20;
        public static final double DRIVING_CURRENT_LIMIT = 40;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

        public static final SimpleMotorFeedforward DRIVE_OPEN_LOOP = new SimpleMotorFeedforward(0.01, 2.03);
    }

    public static final class DriveTrainPhysicsSimulationConstants {
        public static final int SIM_ITERATIONS_PER_ROBOT_PERIOD = 5;

        /* Swerve Module Simulation */
        public static final double DRIVE_MOTOR_FREE_FINAL_SPEED_RPM = 4800;
        public static final DCMotor
                DRIVE_MOTOR = DCMotor.getKrakenX60(1),
                STEER_MOTOR = DCMotor.getFalcon500(1);
        public static final double DRIVE_WHEEL_ROTTER_INERTIA = 0.012;
        public static final double STEER_INERTIA = 0.015;
        public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

        public static final double FLOOR_FRICTION_ACCELERATION_METERS_PER_SEC_SQ = 15;
        public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = Math.toRadians(1200);
        public static final double TIME_CHASSIS_STOPS_ROTATING_NO_POWER_SEC = 0.3;
        public static final double DEFAULT_ROBOT_MASS = 40;
        public static final double DEFAULT_BUMPER_WIDTH_METERS = Units.inchesToMeters(34.5);
        public static final double DEFAULT_BUMPER_LENGTH_METERS = Units.inchesToMeters(36);

        /* https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction */
        public static final double ROBOT_BUMPER_COEFFICIENT_OF_FRICTION = 0.85;
        /* https://en.wikipedia.org/wiki/Coefficient_of_restitution */
        public static final double ROBOT_BUMPER_COEFFICIENT_OF_RESTITUTION = 0.05;

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
    }

    public static final class VisionConfigs {
        public static final AprilTagFieldLayout fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        public static final PhotonPoseEstimator.PoseStrategy poseEstimatorStrategy = PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS;
        /* default standard error for vision observation, if only one apriltag observed */
        public static final double
                TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION = 0.6,
                ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION = 0.5,

                // only do odometry calibration if translational standard error if it is not greater than
                TRANSLATIONAL_STANDARD_ERROR_THRESHOLD = 0.5,
                // only do gyro calibration if rotational standard error is very, very small
                ROTATIONAL_STANDARD_ERROR_THRESHOLD = Math.toRadians(5),

                ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS = 0.02,
                // we trust the IMU very much (recommend 0.1 for Pigeon2, 0.5 for NavX)
                GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS = Math.toRadians(0.1);
    }

    public static Rotation2d toCurrentAllianceRotation(Rotation2d rotationAtBlueSide) {
        final Rotation2d
                yAxis = Rotation2d.fromDegrees(90),
                differenceFromYAxisAtBlueSide = rotationAtBlueSide.minus(yAxis),
                differenceFromYAxisNew = differenceFromYAxisAtBlueSide.times(isSidePresentedAsRed() ? -1:1);
        return yAxis.rotateBy(differenceFromYAxisNew);
    }

    public static Translation2d toCurrentAllianceTranslation(Translation2d translationAtBlueSide) {
        if (isSidePresentedAsRed())
            return new Translation2d(
                    CrescendoField2024Constants.FIELD_WIDTH - translationAtBlueSide.getX(),
                    translationAtBlueSide.getY()
            );
        return translationAtBlueSide;
    }

    public static Translation3d toCurrentAllianceTranslation(Translation3d translation3dAtBlueSide) {
        final Translation2d translation3dAtCurrentAlliance = toCurrentAllianceTranslation(translation3dAtBlueSide.toTranslation2d());
        if (isSidePresentedAsRed())
            return new Translation3d(
                    translation3dAtCurrentAlliance.getX(),
                    translation3dAtCurrentAlliance.getY(),
                    translation3dAtBlueSide.getZ()
            );
        return translation3dAtBlueSide;
    }

    public static Pose2d toCurrentAlliancePose(Pose2d poseAtBlueSide) {
        return new Pose2d(
                toCurrentAllianceTranslation(poseAtBlueSide.getTranslation()),
                toCurrentAllianceRotation(poseAtBlueSide.getRotation())
        );
    }

    public static PathPlannerPath toCurrentAlliancePath(PathPlannerPath pathAtBlueAlliance) {
        return isSidePresentedAsRed() ? pathAtBlueAlliance.flipPath() : pathAtBlueAlliance;
    }

    public static boolean isSidePresentedAsRed() {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get().equals(DriverStation.Alliance.Red);
    }

    public static Rotation2d getDriverStationFacing() {
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
            case Red -> new Rotation2d(Math.PI);
            case Blue -> new Rotation2d(0);
        };
    }
}

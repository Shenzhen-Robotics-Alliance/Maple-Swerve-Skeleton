// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.PhotonPoseEstimator;

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

    public static final class VisionConfigs {
        public static final AprilTagFieldLayout fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        public static final int MINIMUM_TAGS_NUM = 2;
        /* default standard error for vision observation, if only one apriltag observed */
        public static final double
                TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION = 0.3,
                ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION = Math.toRadians(10),

                // only do odometry calibration if translational standard error if it is not greater than
                TRANSLATIONAL_STANDARD_ERROR_THRESHOLD = 0.5,
                // only do gyro calibration if rotational standard error is very, very small
                ROTATIONAL_STANDARD_ERROR_THRESHOLD = Math.toRadians(5),
                ROTATIONAL_ERROR_WITH_GYRO_DISCARD_RESULT = Math.toRadians(15),

                ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS = 0.02,
                // we trust the IMU very much (recommend 0.1 for Pigeon2, 0.5 for NavX)
                GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS = Math.toRadians(0.1);
    }
}

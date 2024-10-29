package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.apriltags.PhotonCameraProperties;

import java.util.List;

public class VisionConstants {
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final double
            /* default standard error for vision observation, if only one apriltag observed */
            TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION = 0.8,
            ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION = Math.toRadians(10),

            TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_MULTITAG = 0.3,
            ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_MULTITAG = Math.toRadians(6),

            /* only do odometry calibration if standard error is not greater than */
            TRANSLATIONAL_STANDARD_DEVS_THRESHOLD_DISCARD_RESULT = 0.4,
            ROTATIONAL_STANDARD_DEVS_THRESHOLD_DISCARD_RESULT = Math.toRadians(20),

            /* standard deviation for odometry and gyros */
            ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS = 0.04,
            GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS = Math.toRadians(0.3);


    public static final List<PhotonCameraProperties> photonVisionCameras = List.of(
            new PhotonCameraProperties(
                    "FrontCam",
                    30, 14, 5,
                    75,
                    0.6, 0.2,
                    1280, 720,
                    new Translation2d(0.330, -0.127), // the outing position of the camera in relative to the robot center
                    0.254, // the mounting height, in meters
                    Rotation2d.fromDegrees(0), // the camera facing, 0 is front, positive is counter-clockwise
                    30, // camera pitch angle, in degrees
                    180 // camera roll angle, 0 for up-right and 180 for upside-down
            ),
            new PhotonCameraProperties(
                    "FrontLeftCam",
                    30, 14, 5,
                    75,
                    0.6, 0.2,
                    1280, 720,
                    new Translation2d(0.229, 0.348),
                    0.2,
                    Rotation2d.fromDegrees(30),
                    30,
                    180 // upside-down
            ),
            new PhotonCameraProperties(
                    "FrontRightCam",
                    30, 14, 5,
                    75,
                    0.6, 0.2,
                    1280, 720,
                    new Translation2d(0.229, -0.348),
                    0.2,
                    Rotation2d.fromDegrees(-30),
                    30,
                    180 // upside-down
            ),
            new PhotonCameraProperties(
                    "BackLeftCam",
                    30, 14, 5,
                    75,
                    0.6, 0.2,
                    1280, 720,
                    new Translation2d(-0.229, 0.330),
                    0.2,
                    Rotation2d.fromDegrees(150),
                    35,
                    180 // upside-down
            ),
            new PhotonCameraProperties(
                    "BackRightCam",
                    30, 14, 5,
                    75,
                    0.6, 0.2,
                    1280, 720,
                    new Translation2d(-0.229, -0.330),
                    0.2,
                    Rotation2d.fromDegrees(-150),
                    35,
                    180 // upside-down
            )
    );
}

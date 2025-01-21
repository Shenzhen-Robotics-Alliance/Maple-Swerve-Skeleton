package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

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
            TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION = 1.5,
            ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION = Math.toRadians(15),
            TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_MULTITAG = 0.5,
            ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_MULTITAG = Math.toRadians(8),
            TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_FOCUSED_TAG = 0.5,
            ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_FOCUSED_TAG = Math.toRadians(8),

            /* standard deviation for odometry and gyros */
            ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS = 0.04,
            GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS = Math.toRadians(0.3);

    public static final List<PhotonCameraProperties> photonVisionCameras = List.of(
            new PhotonCameraProperties(
                    "FrontCam",
                    Hertz.of(30),
                    Milliseconds.of(14),
                    Milliseconds.of(5),
                    Degrees.of(75),
                    0.6,
                    0.2,
                    1280,
                    720,
                    new Translation2d(0.3, 0),
                    Meters.of(0.7),
                    Rotation2d.fromDegrees(0),
                    Degrees.of(40),
                    Degrees.zero()),
            new PhotonCameraProperties(
                    "FrontLeftCam",
                    Hertz.of(30),
                    Milliseconds.of(14),
                    Milliseconds.of(5),
                    Degrees.of(60),
                    0.6,
                    0.2,
                    1280,
                    720,
                    new Translation2d(0.2, 0.3),
                    Meters.of(0.7),
                    Rotation2d.fromDegrees(-35),
                    Degrees.of(-25),
                    Degrees.zero()),
            new PhotonCameraProperties(
                    "FrontRightCam",
                    Hertz.of(30),
                    Milliseconds.of(14),
                    Milliseconds.of(5),
                    Degrees.of(60),
                    0.6,
                    0.2,
                    1280,
                    720,
                    new Translation2d(0.2, -0.3),
                    Meters.of(0.7),
                    Rotation2d.fromDegrees(35),
                    Degrees.of(-25),
                    Degrees.zero()),
            new PhotonCameraProperties(
                    "BackCam",
                    Hertz.of(30),
                    Milliseconds.of(14),
                    Milliseconds.of(5),
                    Degrees.of(75),
                    0.6,
                    0.2,
                    1280,
                    720,
                    new Translation2d(-0.3, 0),
                    Meters.of(0.3),
                    Rotation2d.fromDegrees(180),
                    Degrees.of(40),
                    Degrees.zero()));
    //            List.of(
    //            new PhotonCameraProperties(
    //                    "FrontCam",
    //                    30,
    //                    14,
    //                    5,
    //                    75,
    //                    0.6,
    //                    0.2,
    //                    1280,
    //                    720,
    //                    new Translation2d(
    //                            0.330, -0.127), // the outing position of the camera in relative to the robot center
    //                    0.25, // the mounting height, in meters
    //                    Rotation2d.fromDegrees(0), // the camera facing, 0 is front, positive is counter-clockwise
    //                    24, // camera pitch angle, in degrees
    //                    180 // camera roll angle, 0 for up-right and 180 for upside-down
    //                    ),
    //            new PhotonCameraProperties(
    //                    "FrontLeftCam",
    //                    30,
    //                    14,
    //                    5,
    //                    75,
    //                    0.6,
    //                    0.2,
    //                    1280,
    //                    720,
    //                    new Translation2d(0.229, 0.348),
    //                    0.2,
    //                    Rotation2d.fromDegrees(30),
    //                    30,
    //                    180 // upside-down
    //                    ),
    //            new PhotonCameraProperties(
    //                    "FrontRightCam",
    //                    30,
    //                    14,
    //                    5,
    //                    75,
    //                    0.6,
    //                    0.2,
    //                    1280,
    //                    720,
    //                    new Translation2d(0.229, -0.348),
    //                    0.2,
    //                    Rotation2d.fromDegrees(-30),
    //                    30,
    //                    180 // upside-down
    //                    ),
    //            new PhotonCameraProperties(
    //                    "BackLeftCam",
    //                    30,
    //                    14,
    //                    5,
    //                    75,
    //                    0.6,
    //                    0.2,
    //                    1280,
    //                    720,
    //                    new Translation2d(-0.229, 0.330),
    //                    0.2,
    //                    Rotation2d.fromDegrees(150),
    //                    35,
    //                    180 // upside-down
    //                    ),
    //            new PhotonCameraProperties(
    //                    "BackRightCam",
    //                    30,
    //                    14,
    //                    5,
    //                    75,
    //                    0.6,
    //                    0.2,
    //                    1280,
    //                    720,
    //                    new Translation2d(-0.229, -0.330),
    //                    0.2,
    //                    Rotation2d.fromDegrees(-150),
    //                    35,
    //                    180 // upside-down
    //                    ));
}

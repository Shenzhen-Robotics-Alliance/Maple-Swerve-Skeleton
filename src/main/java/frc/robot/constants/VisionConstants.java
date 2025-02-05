package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.vision.apriltags.PhotonCameraProperties;
import java.util.List;

public class VisionConstants {
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Time POSE_BUFFER_DURATION = Seconds.of(2.5);
    public static final Time ADDITIONAL_LATENCY_COMPENSATION = Milliseconds.of(30);

    // for filtering
    public static final Distance ROBOT_HEIGHT_TOLERANCE = Meters.of(0.15);
    public static final Angle ROBOT_PITCH_TOLERANCE = Degrees.of(10);
    public static final Angle ROBOT_ROLL_TOLERANCE = Degrees.of(10);
    public static final Distance MAX_TAG_DISTANCE = Meters.of(4);
    public static final double MAX_TAG_AMBIGUITY = 0.3;
    public static final Angle MAX_TAG_ANGLE = Degrees.of(65);

    /** Standard errors for single tag vision observations. */
    public static final Distance TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION = Meters.of(2);

    public static final Angle ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION = Degrees.of(60);

    /** Standard errors for multiple-solvePNP observations. */
    public static final Distance TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_MULTITAG = Meters.of(0.5);

    public static final Angle ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_MULTITAG = Degrees.of(8);

    /** Standard errors for focused tag observations. */
    public static final Distance TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_FOCUSED_TAG = Meters.of(0.5);

    public static final Angle ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_FOCUSED_TAG = Degrees.of(8);

    /** Odometry standard errors for the primary pose estimator */
    public static final Distance PRIMARY_ESTIMATOR_ODOMETRY_TRANSLATIONAL_STANDARD_ERROR = Meters.of(0.1);

    public static final Angle PRIMARY_ESTIMATOR_GYRO_ROTATIONAL_STANDARD_ERROR = Degrees.of(0.2);

    /** Odometry standard errors for the secondary (vision-sensitive) pose estimator */
    public static final Distance VISION_SENSITIVE_ESTIMATOR_ODOMETRY_TRANSLATIONAL_STANDARD_ERROR = Meters.of(0.8);

    public static final Angle VISION_SENSITIVE_ESTIMATOR_GYRO_ROTATIONAL_STANDARD_ERROR = Degrees.of(1.5);

    public static final List<PhotonCameraProperties> photonVisionCameras = List.of(
            //            new PhotonCameraProperties(
            //                    "FrontCam",
            //                    Hertz.of(30),
            //                    Milliseconds.of(60),
            //                    Milliseconds.of(5),
            //                    Degrees.of(75),
            //                    0.6,
            //                    0.2,
            //                    1280,
            //                    720,
            //                    new Translation2d(0.3, 0),
            //                    Meters.of(0.7),
            //                    Rotation2d.fromDegrees(0),
            //                    Degrees.of(40),
            //                    Degrees.zero()),
            new PhotonCameraProperties(
                    "FrontLeftCam",
                    Hertz.of(15),
                    Milliseconds.of(80),
                    Milliseconds.of(10),
                    Degrees.of(58),
                    0.9,
                    0.2,
                    1280,
                    800,
                    new Translation2d(0.04, 0.2),
                    Meters.of(0.44),
                    Rotation2d.fromDegrees(0),
                    Degrees.of(-18),
                    Degrees.zero()),
            new PhotonCameraProperties(
                    "FrontRightCam",
                    Hertz.of(15),
                    Milliseconds.of(80),
                    Milliseconds.of(10),
                    Degrees.of(58),
                    0.9,
                    0.2,
                    1280,
                    800,
                    new Translation2d(0.04, -0.2),
                    Meters.of(0.44),
                    Rotation2d.fromDegrees(0),
                    Degrees.of(-18),
                    Degrees.zero())
            //            new PhotonCameraProperties(
            //                    "BackCam",
            //                    Hertz.of(30),
            //                    Milliseconds.of(60),
            //                    Milliseconds.of(5),
            //                    Degrees.of(75),
            //                    0.6,
            //                    0.2,
            //                    1280,
            //                    720,
            //                    new Translation2d(-0.3, 0),
            //                    Meters.of(0.3),
            //                    Rotation2d.fromDegrees(180),
            //                    Degrees.of(40),
            //                    Degrees.zero())
            );
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

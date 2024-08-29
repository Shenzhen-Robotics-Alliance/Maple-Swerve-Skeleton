package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.apriltags.PhotonCameraProperties;

import java.util.List;

public class VisionConstants {
    public static final List<PhotonCameraProperties> photonVisionCameras = List.of(
            new PhotonCameraProperties(
                    "FrontCam",
                        50, 12, 5,
                       60.12,
                        0.75, 0.25,
                        1280, 720,
                        new Translation2d(0.2, 0),
                        0.3,
                        Rotation2d.fromDegrees(0),
                        24,
                        180 // upside-down
            ),
            new PhotonCameraProperties(
                    "FrontLeftCam",
                    50, 12, 5,
                    60.12,
                    0.75, 0.25,
                    1280, 720,
                    new Translation2d(0.2, 0.15),
                    0.3,
                    Rotation2d.fromDegrees(35),
                    30,
                    180 // upside-down
            ),
            new PhotonCameraProperties(
                    "FrontRightCam",
                    50, 12, 5,
                    60.12,
                    0.75, 0.25,
                    1280, 720,
                    new Translation2d(0.2, -0.15),
                    0.3,
                    Rotation2d.fromDegrees(-35),
                    30,
                    180 // upside-down
            ),
            new PhotonCameraProperties(
                    "BackLeftCam",
                    50, 12, 5,
                    60.12,
                    0.75, 0.25,
                    1280, 720,
                    new Translation2d(-0.2, 0.15),
                    0.3,
                    Rotation2d.fromDegrees(60),
                    30,
                    180 // upside-down
            ),
            new PhotonCameraProperties(
                    "BackRightCam",
                    50, 12, 5,
                    60.12,
                    0.75, 0.25,
                    1280, 720,
                    new Translation2d(-0.2, -0.15),
                    0.3,
                    Rotation2d.fromDegrees(-60),
                    30,
                    180 // upside-down
            )
    );
}

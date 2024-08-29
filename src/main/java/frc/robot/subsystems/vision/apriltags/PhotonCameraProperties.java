package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.CustomConfigs.MapleConfigFile;
import org.photonvision.simulation.SimCameraProperties;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.JSONObject;
import org.json.simple.JSONArray;

public class PhotonCameraProperties {
    public final String name;
    public final double frameRate, averageLatencyMS, latencyStandardDeviationMS,
            calibrationAverageErrorPixel, calibrationErrorStandardDeviation;
    public final Rotation2d cameraFOVDiag;
    public final int captureWidthPixels, captureHeightPixels;
    public final Transform3d robotToCamera;

    public PhotonCameraProperties(
            String name,
            double frameRate, double averageLatencyMS, double latencyStandardDeviationMS,
            double cameraFOVDegreesDiag,
            double calibrationAverageErrorPixel, double calibrationErrorStandardDeviation,
            int captureWidthPixels, int captureHeightPixels,
            Translation2d mountPositionOnRobotMeters,
            double mountHeightMeters,
            Rotation2d cameraFacing,
            double cameraPitchDegrees,
            double rotateImageDegrees
    ) {
        this(
                name, frameRate, averageLatencyMS, latencyStandardDeviationMS, Rotation2d.fromDegrees(cameraFOVDegreesDiag), calibrationAverageErrorPixel, calibrationErrorStandardDeviation, captureWidthPixels, captureHeightPixels,
                new Transform3d(mountPositionOnRobotMeters.getX(), mountPositionOnRobotMeters.getY(), mountHeightMeters,
                        new Rotation3d(
                                0,
                                -Math.toRadians(cameraPitchDegrees),
                                cameraFacing.getRadians()
                        ))
                        .plus(new Transform3d(new Translation3d(), new Rotation3d(
                                Math.toRadians(rotateImageDegrees), 0, 0)))
        );
    }

    public PhotonCameraProperties(
            String name,
            double frameRate, double averageLatencyMS, double latencyStandardDeviationMS,
            Rotation2d cameraFOVDiag,
            double calibrationAverageErrorPixel, double calibrationErrorStandardDeviation,
            int captureWidthPixels, int captureHeightPixels,
            Transform3d robotToCamera
    ) {
        this.name = name;
        this.frameRate = frameRate;
        this.averageLatencyMS = averageLatencyMS;
        this.latencyStandardDeviationMS = latencyStandardDeviationMS;
        this.cameraFOVDiag = cameraFOVDiag;
        this.calibrationAverageErrorPixel = calibrationAverageErrorPixel;
        this.calibrationErrorStandardDeviation = calibrationErrorStandardDeviation;
        this.captureWidthPixels = captureWidthPixels;
        this.captureHeightPixels = captureHeightPixels;
        this.robotToCamera = robotToCamera;

        System.out.println("Created photon camera: " + name + " on robot");
        System.out.println("Advantage Scope Asset String:\n" + toAdvantageScopeAssetFixedCameraConfigurationJsonString());
    }

    public SimCameraProperties getSimulationProperties() {
        final SimCameraProperties cameraProperties = new SimCameraProperties();
        cameraProperties.setFPS(frameRate);
        cameraProperties.setAvgLatencyMs(averageLatencyMS);
        cameraProperties.setLatencyStdDevMs(latencyStandardDeviationMS);
        cameraProperties.setCalibration(captureWidthPixels, captureHeightPixels, cameraFOVDiag);
        cameraProperties.setCalibError(calibrationAverageErrorPixel, calibrationErrorStandardDeviation);
        return cameraProperties;
    }

    public static List<PhotonCameraProperties> loadCamerasPropertiesFromConfig(String configName) {
        final MapleConfigFile configFile;
        try {
            configFile = MapleConfigFile.fromDeployedConfig("PhotonCamerasProperties", configName);
        } catch (IOException e) {
            DriverStation.reportError(
                    "cannot find camera properties config file: "
                            + configName
                            + " from deploy directory",
                    true);
            return new ArrayList<>();
        }
        return loadCamerasPropertiesFromConfig(configFile);
    }

    public static List<PhotonCameraProperties> loadCamerasPropertiesFromConfig(MapleConfigFile configFile) {
        MapleConfigFile.ConfigBlock generalBlock = configFile.getBlock("GeneralInfo");
        final int cameraAmount = generalBlock.getIntConfig("camerasAmount");
        final double cameraFPS = generalBlock.getDoubleConfig("cameraFPS"),
                averageLatencyMS = generalBlock.getDoubleConfig("averageLatencyMS"),
                latencyStandardDeviationMS = generalBlock.getDoubleConfig("latencyStandardDeviationMS");

        final List<PhotonCameraProperties> cameraProperties = new ArrayList<>(cameraAmount);

        for (int i = 0; i < cameraAmount ; i++) {
            final MapleConfigFile.ConfigBlock cameraBlock = configFile.getBlock("Camera"+i);
            cameraProperties.add(loadSingleCameraPropertyFromBlock(
                    cameraBlock, cameraFPS, averageLatencyMS, latencyStandardDeviationMS
            ));
        }

        return cameraProperties;
    }

    private static PhotonCameraProperties loadSingleCameraPropertyFromBlock(MapleConfigFile.ConfigBlock cameraBlock, double cameraFPS, double averageLatencyMS, double latencyStandardDeviationMS) {
        final Transform3d robotToCameraInstallation = new Transform3d(
                new Translation3d(
                        cameraBlock.getDoubleConfig("mountPositionToRobotCenterForwardsMeters"),
                        cameraBlock.getDoubleConfig("mountPositionToRobotCenterLeftwardsMeters"),
                        cameraBlock.getDoubleConfig("mountHeightMeters")
                ),
                new Rotation3d(
                        0,
                        -Math.toRadians(cameraBlock.getDoubleConfig("cameraPitchDegrees")),
                        Math.toRadians(cameraBlock.getDoubleConfig("cameraYawDegrees"))
                ));
        final double cameraRollDeg = switch (cameraBlock.getIntConfig("captureOrientation")) {
            case 1 -> 180;
            case 2 -> -90;
            case 3 -> 90;
            default -> 0;
        };
        final Transform3d cameraInstallationToCapturedImage = new Transform3d(
                        new Translation3d(),
                        new Rotation3d(
                                Math.toRadians(cameraRollDeg),
                                0, 0
                        ));
        return new PhotonCameraProperties(
                cameraBlock.getStringConfig("name"),
                cameraFPS,
                averageLatencyMS,
                latencyStandardDeviationMS,
                Rotation2d.fromDegrees(cameraBlock.getDoubleConfig("cameraFOVDegrees")),
                cameraBlock.getDoubleConfig("calibrationAverageErrorPixel"),
                cameraBlock.getDoubleConfig("calibrationErrorStandardDeviationPixel"),
                cameraBlock.getIntConfig("captureWidthPixels"),
                cameraBlock.getIntConfig("captureHeightPixels"),
                robotToCameraInstallation.plus(cameraInstallationToCapturedImage)
        );
    }

    public static MapleConfigFile createConfigFileForCameras(String name, int camerasAmount, double cameraFPS, double averageLatencyMS, double latencyStandardDeviationMS, int cameraWidthPixels, int cameraHeightPixels, double defaultCameraFOVDegrees) {
        final MapleConfigFile configFile = new MapleConfigFile("PhotonCamerasProperties", name);
        final MapleConfigFile.ConfigBlock generalBlock = configFile.getBlock("GeneralInfo");

        generalBlock.putIntConfig("camerasAmount", camerasAmount);
        generalBlock.putDoubleConfig("cameraFPS", cameraFPS);
        generalBlock.putDoubleConfig("averageLatencyMS", averageLatencyMS);
        generalBlock.putDoubleConfig("latencyStandardDeviationMS", latencyStandardDeviationMS);

        for (int i = 0; i < camerasAmount; i++) {
            createConfigBlockForCamera(
                    configFile.getBlock("Camera"+i),
                    cameraWidthPixels,cameraHeightPixels,defaultCameraFOVDegrees
            );
        }

        return configFile;
    }

    private static void createConfigBlockForCamera(MapleConfigFile.ConfigBlock cameraBlock, int cameraWidthPixels, int cameraHeightPixels, double cameraFOVDegrees) {
        cameraBlock.putStringConfig("name", "YOUR_CAMERA_NAME");
        cameraBlock.putIntConfig("captureWidthPixels", cameraWidthPixels);
        cameraBlock.putIntConfig("captureHeightPixels", cameraHeightPixels);
        cameraBlock.putDoubleConfig("cameraFOVDegrees", cameraFOVDegrees);
        cameraBlock.putDoubleConfig("calibrationAverageErrorPixel", 0.35);
        cameraBlock.putDoubleConfig("calibrationErrorStandardDeviationPixel", 0.1);
        cameraBlock.putDoubleConfig("mountPositionToRobotCenterForwardsMeters", 0);
        cameraBlock.putDoubleConfig("mountPositionToRobotCenterLeftwardsMeters", 0);
        cameraBlock.putDoubleConfig("mountHeightMeters", 0);
        cameraBlock.putDoubleConfig("cameraPitchDegrees", 0);
        cameraBlock.putDoubleConfig("cameraYawDegrees", 0);
    }

    @Override
    public String toString() {
        return String.format("""
                        PhotonCameraProperties of camera %s {,
                          frameRate: %.2f,
                          averageLatencyMS: %.2f,
                          latencyStandardDeviationMS: %.2f,
                          calibrationAverageErrorPixel: %.2f,
                          calibrationErrorStandardDeviation: %.2f,
                          cameraFOVDiag: %s,
                          captureWidthPixels: %d,
                          captureHeightPixels: %d,
                          cameraToRobot: %s
                        }""",
                name, frameRate, averageLatencyMS, latencyStandardDeviationMS,
                calibrationAverageErrorPixel, calibrationErrorStandardDeviation,
                cameraFOVDiag.toString(), captureWidthPixels, captureHeightPixels, robotToCamera.toString());
    }

    @SuppressWarnings("unchecked")
    public String toAdvantageScopeAssetFixedCameraConfigurationJsonString() {
        // Create the JSON object
        JSONObject jsonObject = new JSONObject();

        // Add the name
        jsonObject.put("name", this.name);

        // Add the rotations
        JSONArray rotations = new JSONArray();
        JSONObject rotationY = new JSONObject();
        rotationY.put("axis", "y");
        rotationY.put("degrees", Math.toDegrees(robotToCamera.getRotation().getY()));
        rotations.add(rotationY);
        JSONObject rotationZ = new JSONObject();
        rotationZ.put("axis", "z");
        rotationZ.put("degrees", Math.toDegrees(robotToCamera.getRotation().getZ()));
        rotations.add(rotationZ);
        jsonObject.put("rotations", rotations);

        // Add the position
        JSONArray position = new JSONArray();
        position.add(robotToCamera.getX());
        position.add(robotToCamera.getY());
        position.add(robotToCamera.getZ());
        jsonObject.put("position", position);

        // Add the resolution
        JSONArray resolution = new JSONArray();
        resolution.add(captureWidthPixels);
        resolution.add(captureHeightPixels);
        jsonObject.put("resolution", resolution);

        // Add the FOV
        jsonObject.put("fov", cameraFOVDiag.getDegrees());

        // Return the JSON string
        return jsonObject.toJSONString(); // Convert the JSONObject to a JSON string
    }
}

package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.*;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.photonvision.simulation.SimCameraProperties;

public class PhotonCameraProperties {
    public final String name;
    public final double frameRate,
            averageLatencyMS,
            latencyStandardDeviationMS,
            calibrationAverageErrorPixel,
            calibrationErrorStandardDeviation;
    public final Rotation2d cameraFOVDiag;
    public final int captureWidthPixels, captureHeightPixels;
    public final Transform3d robotToCamera;

    public PhotonCameraProperties(
            String name,
            double frameRate,
            double averageLatencyMS,
            double latencyStandardDeviationMS,
            double cameraFOVDegreesDiag,
            double calibrationAverageErrorPixel,
            double calibrationErrorStandardDeviation,
            int captureWidthPixels,
            int captureHeightPixels,
            Translation2d mountPositionOnRobotMeters,
            double mountHeightMeters,
            Rotation2d cameraFacing,
            double cameraPitchDegrees,
            double rotateImageDegrees) {
        this(
                name,
                frameRate,
                averageLatencyMS,
                latencyStandardDeviationMS,
                Rotation2d.fromDegrees(cameraFOVDegreesDiag),
                calibrationAverageErrorPixel,
                calibrationErrorStandardDeviation,
                captureWidthPixels,
                captureHeightPixels,
                new Transform3d(
                                mountPositionOnRobotMeters.getX(),
                                mountPositionOnRobotMeters.getY(),
                                mountHeightMeters,
                                new Rotation3d(0, -Math.toRadians(cameraPitchDegrees), cameraFacing.getRadians()))
                        .plus(new Transform3d(
                                new Translation3d(), new Rotation3d(Math.toRadians(rotateImageDegrees), 0, 0))));
    }

    public PhotonCameraProperties(
            String name,
            double frameRate,
            double averageLatencyMS,
            double latencyStandardDeviationMS,
            Rotation2d cameraFOVDiag,
            double calibrationAverageErrorPixel,
            double calibrationErrorStandardDeviation,
            int captureWidthPixels,
            int captureHeightPixels,
            Transform3d robotToCamera) {
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
        System.out.println(
                "Advantage Scope Asset String:\n" + toAdvantageScopeAssetFixedCameraConfigurationJsonString());
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

    @Override
    public String toString() {
        return String.format(
                """
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
                name,
                frameRate,
                averageLatencyMS,
                latencyStandardDeviationMS,
                calibrationAverageErrorPixel,
                calibrationErrorStandardDeviation,
                cameraFOVDiag.toString(),
                captureWidthPixels,
                captureHeightPixels,
                robotToCamera.toString());
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

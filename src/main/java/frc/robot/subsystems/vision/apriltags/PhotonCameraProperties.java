package frc.robot.subsystems.vision.apriltags;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.photonvision.simulation.SimCameraProperties;

public class PhotonCameraProperties {
    public final String name;
    public final Frequency frameRate;
    public final Time averageLatency, latencyStandardDeviation;
    public final double calibrationAverageErrorPixel, calibrationErrorStandardDeviation;
    public final Rotation2d cameraFOVDiag;
    public final int captureWidthPixels, captureHeightPixels;
    public final Transform3d robotToCamera;

    @Deprecated
    public PhotonCameraProperties(
            String name,
            double frameRateFPS,
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
                Hertz.of(frameRateFPS),
                Milliseconds.of(averageLatencyMS),
                Milliseconds.of(latencyStandardDeviationMS),
                Degrees.of(cameraFOVDegreesDiag),
                calibrationAverageErrorPixel,
                calibrationErrorStandardDeviation,
                captureWidthPixels,
                captureHeightPixels,
                mountPositionOnRobotMeters,
                Meters.of(mountHeightMeters),
                cameraFacing,
                Degrees.of(cameraPitchDegrees),
                Degrees.of(rotateImageDegrees));
    }

    public PhotonCameraProperties(
            String name,
            Frequency frameRate,
            Time averageLatency,
            Time latencyStandardDeviation,
            Angle cameraFOVDiag,
            double calibrationAverageErrorPixel,
            double calibrationErrorStandardDeviation,
            int captureWidthPixels,
            int captureHeightPixels,
            Translation2d mountPositionOnRobotMeters,
            Distance mountHeight,
            Rotation2d cameraFacing,
            Angle cameraPitch,
            Angle rotateImage) {
        this(
                name,
                frameRate,
                averageLatency,
                latencyStandardDeviation,
                new Rotation2d(cameraFOVDiag),
                calibrationAverageErrorPixel,
                calibrationErrorStandardDeviation,
                captureWidthPixels,
                captureHeightPixels,
                new Transform3d(
                                mountPositionOnRobotMeters.getX(),
                                mountPositionOnRobotMeters.getY(),
                                mountHeight.in(Meters),
                                new Rotation3d(0, -cameraPitch.in(Radians), cameraFacing.getRadians()))
                        .plus(new Transform3d(new Translation3d(), new Rotation3d(rotateImage.in(Radians), 0, 0))));
    }

    public PhotonCameraProperties(
            String name,
            Frequency frameRate,
            Time averageLatency,
            Time latencyStandardDeviation,
            Rotation2d cameraFOVDiag,
            double calibrationAverageErrorPixel,
            double calibrationErrorStandardDeviation,
            int captureWidthPixels,
            int captureHeightPixels,
            Transform3d robotToCamera) {
        this.name = name;
        this.frameRate = frameRate;
        this.averageLatency = averageLatency;
        this.latencyStandardDeviation = latencyStandardDeviation;
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
        cameraProperties.setFPS(frameRate.in(Hertz));
        cameraProperties.setAvgLatencyMs(averageLatency.in(Millisecond));
        cameraProperties.setLatencyStdDevMs(latencyStandardDeviation.in(Millisecond));
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
                frameRate.in(Hertz),
                averageLatency.in(Milliseconds),
                latencyStandardDeviation.in(Milliseconds),
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

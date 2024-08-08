package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.Config.MapleInterpolationTable;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;

import static frc.robot.utils.Config.MapleInterpolationTable.Variable;

public class MapleShooterOptimization {
    public static final class ShooterState {
        public final double shooterAngleDegrees;
        public final double shooterAngleChangeRateDegreesPerSecond;
        public final double shooterRPM;
        public final double shooterRPMChangeRateRPMPerSeconds;

        private ShooterState(double shooterAngleDegrees, double shooterAngleChangeRateDegreesPerSecond, double shooterRPM, double shooterRPMChangeRateRPMPerSeconds) {
            this.shooterAngleDegrees = shooterAngleDegrees;
            this.shooterAngleChangeRateDegreesPerSecond = shooterAngleChangeRateDegreesPerSecond;
            this.shooterRPM = shooterRPM;
            this.shooterRPMChangeRateRPMPerSeconds = shooterRPMChangeRateRPMPerSeconds;
        }

        @Override
        public String toString() {
            return "ShootingState{" +
                    "shooterAngleDegrees=" + shooterAngleDegrees +
                    ", shooterAngleChangeRateDegreesPerSecond=" + shooterAngleChangeRateDegreesPerSecond +
                    ", shooterRPM=" + shooterRPM +
                    ", shooterRPMChangeRateRPMPerSeconds=" + shooterRPMChangeRateRPMPerSeconds +
                    '}';
        }

        public void log(String logPath) {
            Logger.recordOutput(logPath + "ShooterAngleDegrees", shooterAngleDegrees);
            Logger.recordOutput(logPath + "ShooterAngleChangeRateDegreesPerSecond", shooterAngleChangeRateDegreesPerSecond);
            Logger.recordOutput(logPath + "ShooterRPM", shooterRPM);
            Logger.recordOutput(logPath + "ShooterRPMChangeRateRPMPerSeconds", shooterRPMChangeRateRPMPerSeconds);
        }
    }

    private final String name;
    private final MapleInterpolationTable table;
    private final double minShootingDistance, maxShootingDistance;
    public MapleShooterOptimization(String name, double[] distancesToTargetsMeters, double[] shooterAngleDegrees, double[] shooterRPM, double[] projectileFlightTimeSeconds) {
        this(name, new MapleInterpolationTable(
                name,
                new Variable("Distance-To-Target", distancesToTargetsMeters),
                new Variable("Shooter-Angle-Degrees", shooterAngleDegrees),
                new Variable("Shooter-RPM", shooterRPM),
                new Variable("Flight-Time", projectileFlightTimeSeconds)
        ));
    }

    private MapleShooterOptimization(String name, MapleInterpolationTable table) {
        this.name = name;
        this.table = table;

        this.minShootingDistance = table.minX;
        this.maxShootingDistance = table.maxX;
    }

    public double getFlightTimeSeconds(Translation2d targetPosition, Translation2d robotPosition) {
        final double distanceToTargetMeters = targetPosition.getDistance(robotPosition);
        return table.interpolateVariable("Flight-Time", distanceToTargetMeters);
    }

    public ShooterState getOptimizedShootingState(Translation2d targetPosition, Translation2d robotPosition, ChassisSpeeds robotVelocityFieldRelative) {
        final double flightTimeSeconds = getFlightTimeSeconds(targetPosition, robotPosition);
        final Translation2d robotNewPosition = robotPosition.plus(new Translation2d(
                robotVelocityFieldRelative.vxMetersPerSecond * flightTimeSeconds,
                robotVelocityFieldRelative.vyMetersPerSecond * flightTimeSeconds
        ));
        final double newDistanceToTarget = targetPosition.getDistance(robotNewPosition);
        final Rotation2d chassisSpeedsDirection = new Rotation2d(robotVelocityFieldRelative.vxMetersPerSecond, robotVelocityFieldRelative.vyMetersPerSecond),
                targetToChassisHeading = robotPosition.minus(targetPosition).getAngle();
        final double distanceToTargetChangingRate = targetToChassisHeading.minus(chassisSpeedsDirection).getCos()
                * Math.hypot(robotVelocityFieldRelative.vyMetersPerSecond, robotVelocityFieldRelative.vxMetersPerSecond);

        Logger.recordOutput("ShooterStateOptimization/distance to target change rate", distanceToTargetChangingRate);
        final double shooterAngleDeg = table.interpolateVariable("Shooter-Angle-Degrees", newDistanceToTarget),
                shooterAngleDegChangeRateToDistance = table.findDerivative("Shooter-Angle-Degrees", newDistanceToTarget, 0.1),
                shooterAngleDegChangeRateDegPerSec = shooterAngleDegChangeRateToDistance * distanceToTargetChangingRate,
                shooterRPM = table.interpolateVariable("Shooter-RPM", newDistanceToTarget),
                shooterRPMChangeRateToDistance = table.findDerivative("Shooter-RPM", newDistanceToTarget, 0.1),
                shooterRPMChangeRateRPMPerSec = shooterRPMChangeRateToDistance * distanceToTargetChangingRate;

        return new ShooterState(
                shooterAngleDeg,
                shooterAngleDegChangeRateDegPerSec,
                shooterRPM,
                shooterRPMChangeRateRPMPerSec
        );
    }

    public boolean isTargetInRange(Translation2d targetPosition, Translation2d robotPosition) {
        final double distanceToTarget = targetPosition.getDistance(robotPosition);
        return minShootingDistance <= distanceToTarget && maxShootingDistance >= distanceToTarget;
    }

    public static MapleShooterOptimization fromDeployDirectory(String name) throws IOException {
        final MapleInterpolationTable interpolationTable = MapleInterpolationTable.fromConfigFile(
                MapleConfigFile.fromDeployedConfig("ShooterOptimization", name)
        );
        return new MapleShooterOptimization(name, interpolationTable);
    }

    public void saveConfigsToDeploy() {
        table.toConfigFile("ShooterOptimization").saveConfigToUSBSafe();
    }
}

package frc.robot.utils;

import static frc.robot.utils.CustomConfigs.MapleInterpolationTable.Variable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.CustomConfigs.MapleInterpolationTable;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class MapleShooterOptimization {
    public static final class ShooterState {
        public final double shooterAngleDegrees;
        public final double shooterAngleChangeRateDegreesPerSecond;
        public final double shooterRPM;
        public final double shooterRPMChangeRateRPMPerSeconds;

        private ShooterState(
                double shooterAngleDegrees,
                double shooterAngleChangeRateDegreesPerSecond,
                double shooterRPM,
                double shooterRPMChangeRateRPMPerSeconds) {
            this.shooterAngleDegrees = shooterAngleDegrees;
            this.shooterAngleChangeRateDegreesPerSecond = shooterAngleChangeRateDegreesPerSecond;
            this.shooterRPM = shooterRPM;
            this.shooterRPMChangeRateRPMPerSeconds = shooterRPMChangeRateRPMPerSeconds;
        }

        @Override
        public String toString() {
            return "ShootingState{"
                    + "shooterAngleDegrees="
                    + shooterAngleDegrees
                    + ", shooterAngleChangeRateDegreesPerSecond="
                    + shooterAngleChangeRateDegreesPerSecond
                    + ", shooterRPM="
                    + shooterRPM
                    + ", shooterRPMChangeRateRPMPerSeconds="
                    + shooterRPMChangeRateRPMPerSeconds
                    + '}';
        }

        public void log(String logPath) {
            Logger.recordOutput(logPath + "ShooterAngleDegrees", shooterAngleDegrees);
            Logger.recordOutput(
                    logPath + "ShooterAngleChangeRateDegreesPerSecond", shooterAngleChangeRateDegreesPerSecond);
            Logger.recordOutput(logPath + "ShooterRPM", shooterRPM);
            Logger.recordOutput(logPath + "ShooterRPMChangeRateRPMPerSeconds", shooterRPMChangeRateRPMPerSeconds);
        }
    }

    private final String name;
    private final MapleInterpolationTable table;
    private final double minShootingDistance, maxShootingDistance;

    public MapleShooterOptimization(
            String name,
            double[] distancesToTargetsMeters,
            double[] shooterAngleDegrees,
            double[] shooterRPM,
            double[] projectileFlightTimeSeconds) {
        this(
                name,
                new MapleInterpolationTable(
                        name,
                        new Variable("Distance-To-Target", distancesToTargetsMeters),
                        new Variable("Shooter-Angle-Degrees", shooterAngleDegrees),
                        new Variable("Shooter-RPM", shooterRPM),
                        new Variable("Flight-Time", projectileFlightTimeSeconds)));
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

    public Rotation2d getShooterFacing(
            Translation2d targetPosition, Translation2d robotPosition, ChassisSpeeds robotVelocityFieldRelative) {
        final double flightTime = getFlightTimeSeconds(targetPosition, robotPosition);
        final Translation2d robotPositionAfterFlightTime = robotPosition.plus(new Translation2d(
                robotVelocityFieldRelative.vxMetersPerSecond * flightTime,
                robotVelocityFieldRelative.vyMetersPerSecond * flightTime));

        return targetPosition.minus(robotPositionAfterFlightTime).getAngle();
    }

    public ShooterState getOptimizedShootingState(
            Translation2d targetPosition, Translation2d robotPosition, ChassisSpeeds robotVelocityFieldRelative) {
        final double flightTimeSeconds = getFlightTimeSeconds(targetPosition, robotPosition);
        final Translation2d robotNewPosition = robotPosition.plus(new Translation2d(
                robotVelocityFieldRelative.vxMetersPerSecond * flightTimeSeconds,
                robotVelocityFieldRelative.vyMetersPerSecond * flightTimeSeconds));
        final double newDistanceToTarget = targetPosition.getDistance(robotNewPosition);
        final Rotation2d
                chassisSpeedsDirection =
                        new Rotation2d(
                                robotVelocityFieldRelative.vxMetersPerSecond,
                                robotVelocityFieldRelative.vyMetersPerSecond),
                targetToChassisHeading = robotPosition.minus(targetPosition).getAngle();
        final double distanceToTargetChangingRate =
                targetToChassisHeading.minus(chassisSpeedsDirection).getCos()
                        * Math.hypot(
                                robotVelocityFieldRelative.vyMetersPerSecond,
                                robotVelocityFieldRelative.vxMetersPerSecond);

        Logger.recordOutput("ShooterStateOptimization/distance to target change rate", distanceToTargetChangingRate);
        final double shooterAngleDeg = table.interpolateVariable("Shooter-Angle-Degrees", newDistanceToTarget),
                shooterAngleDegChangeRateToDistance =
                        table.findDerivative("Shooter-Angle-Degrees", newDistanceToTarget, 0.1),
                shooterAngleDegChangeRateDegPerSec = shooterAngleDegChangeRateToDistance * distanceToTargetChangingRate,
                shooterRPM = table.interpolateVariable("Shooter-RPM", newDistanceToTarget),
                shooterRPMChangeRateToDistance = table.findDerivative("Shooter-RPM", newDistanceToTarget, 0.1),
                shooterRPMChangeRateRPMPerSec = shooterRPMChangeRateToDistance * distanceToTargetChangingRate;

        return new ShooterState(
                shooterAngleDeg, shooterAngleDegChangeRateDegPerSec, shooterRPM, shooterRPMChangeRateRPMPerSec);
    }

    public boolean isTargetInRange(Translation2d targetPosition, Translation2d robotPosition) {
        final double distanceToTarget = targetPosition.getDistance(robotPosition);
        return minShootingDistance <= distanceToTarget && maxShootingDistance >= distanceToTarget;
    }

    public static class ChassisAimAtSpeakerDuringAuto extends Command {
        private final AtomicReference<Optional<Rotation2d>> rotationalTargetOverride;
        private final Supplier<Translation2d> targetPositionSupplier;
        private final HolonomicDriveSubsystem driveSubsystem;
        private final MapleShooterOptimization shooterOptimization;

        public ChassisAimAtSpeakerDuringAuto(
                AtomicReference<Optional<Rotation2d>> rotationalTargetOverride,
                Supplier<Translation2d> targetPositionSupplier,
                HolonomicDriveSubsystem driveSubsystem,
                MapleShooterOptimization shooterOptimization) {
            this.rotationalTargetOverride = rotationalTargetOverride;
            this.targetPositionSupplier = targetPositionSupplier;
            this.driveSubsystem = driveSubsystem;
            this.shooterOptimization = shooterOptimization;
        }

        Rotation2d desiredChassisFacing = new Rotation2d();
        boolean complete = false;

        @Override
        public void initialize() {
            complete = false;
        }

        @Override
        public void execute() {
            desiredChassisFacing = shooterOptimization.getShooterFacing(
                    targetPositionSupplier.get(),
                    driveSubsystem.getPose().getTranslation(),
                    driveSubsystem.getMeasuredChassisSpeedsFieldRelative());
            rotationalTargetOverride.set(Optional.of(desiredChassisFacing));
            complete = driveSubsystem.getFacing().minus(desiredChassisFacing).getDegrees() < 3;
        }

        @Override
        public void end(boolean interrupted) {
            rotationalTargetOverride.set(Optional.empty());
        }

        public boolean aimComplete() {
            return complete;
        }
    }

    public ChassisAimAtSpeakerDuringAuto chassisAimAtSpeakerDuringAuto(
            AtomicReference<Optional<Rotation2d>> rotationalTargetOverride,
            Supplier<Translation2d> targetPositionSupplier,
            HolonomicDriveSubsystem driveSubsystem) {
        return new ChassisAimAtSpeakerDuringAuto(
                rotationalTargetOverride, targetPositionSupplier, driveSubsystem, this);
    }
}

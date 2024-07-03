package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SkiddingEstimator {
    /**
     * the method comes from 1690's <a
     * href="https://youtu.be/N6ogT5DjGOk?feature=shared&t=1674">online software session</a> gets the
     * skidding ratio from the latest , that can be used to determine how much the chassis is skidding
     * the skidding ratio is defined as the ratio between the maximum and minimum magnitude of the
     * "translational" part of the speed of the modules
     *
     * @param swerveStatesMeasured  the swerve states measured from the modules
     * @param swerveDriveKinematics the kinematics
     * @return the skidding ratio, maximum/minimum, ranges from [1,INFINITY)
     */
    public static double getSkiddingRatio(
            SwerveModuleState[] swerveStatesMeasured, SwerveDriveKinematics swerveDriveKinematics) {
        final double angularVelocityOmegaMeasured =
                swerveDriveKinematics.toChassisSpeeds(swerveStatesMeasured).omegaRadiansPerSecond;
        final SwerveModuleState[] swerveStatesRotationalPart =
                swerveDriveKinematics.toSwerveModuleStates(
                        new ChassisSpeeds(0, 0, angularVelocityOmegaMeasured));
        final double[] swerveStatesTranslationalPartMagnitudes =
                new double[swerveStatesMeasured.length];

        for (int i = 0; i < swerveStatesMeasured.length; i++) {
            final Translation2d
                    swerveStateMeasuredAsVector = convertSwerveStateToVelocityVector(swerveStatesMeasured[i]),
                    swerveStatesRotationalPartAsVector =
                            convertSwerveStateToVelocityVector(swerveStatesRotationalPart[i]),
                    swerveStatesTranslationalPartAsVector =
                            swerveStateMeasuredAsVector.minus(swerveStatesRotationalPartAsVector);
            swerveStatesTranslationalPartMagnitudes[i] = swerveStatesTranslationalPartAsVector.getNorm();
        }

        double maximumTranslationalSpeed = 0, minimumTranslationalSpeed = Double.POSITIVE_INFINITY;
        for (double translationalSpeed : swerveStatesTranslationalPartMagnitudes) {
            maximumTranslationalSpeed = Math.max(maximumTranslationalSpeed, translationalSpeed);
            minimumTranslationalSpeed = Math.min(minimumTranslationalSpeed, translationalSpeed);
        }

        return maximumTranslationalSpeed / minimumTranslationalSpeed;
    }

    private static Translation2d convertSwerveStateToVelocityVector(
            SwerveModuleState swerveModuleState) {
        return new Translation2d(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
    }

    /**
     * Estimates the skidding of the chassis from the latest measured swerve states
     *
     * @return the standard deviation of the current swerve state, from the ideal swerve state
     */
    public static double getSkiddingStandardDeviation(
            SwerveModuleState[] measuredSwerveStates, SwerveDriveKinematics swerveDriveKinematics) {
        final ChassisSpeeds measuredChassisSpeed =
                swerveDriveKinematics.toChassisSpeeds(measuredSwerveStates);
        final SwerveModuleState[] idealSwerveStatesGivenNoSkidding =
                swerveDriveKinematics.toSwerveModuleStates(measuredChassisSpeed);

        double totalSquaredDeviation = 0;
        for (int i = 0; i < 4; i++)
            totalSquaredDeviation +=
                    getSquaredDifferenceBetweenTwoSwerveStates(
                            measuredSwerveStates[i], idealSwerveStatesGivenNoSkidding[i]);

        final double variance = totalSquaredDeviation / 4;
        return Math.sqrt(variance);
    }

    /**
     * gets the squared difference between the velocity vector of two swerve states
     *
     * @return the squared difference, in (meters/seconds)^2,
     */
    private static double getSquaredDifferenceBetweenTwoSwerveStates(
            SwerveModuleState swerveModuleState1, SwerveModuleState swerveModuleState2) {
        final Translation2d
                swerveState1VelocityVector =
                new Translation2d(swerveModuleState1.speedMetersPerSecond, swerveModuleState1.angle),
                swerveState2VelocityVector =
                        new Translation2d(swerveModuleState2.speedMetersPerSecond, swerveModuleState2.angle);

        return Math.pow(swerveState1VelocityVector.getDistance(swerveState2VelocityVector), 2);
    }
}

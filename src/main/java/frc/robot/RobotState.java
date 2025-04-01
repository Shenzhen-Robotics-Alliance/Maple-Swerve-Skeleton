package frc.robot;

// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveControlLoops.*;
import static frc.robot.constants.DriveTrainConstants.*;
import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.vision.apriltags.MapleMultiTagPoseEstimator;
import frc.robot.utils.AlertsManager;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public class RobotState {
    private final Alert visionNoResultAlert = AlertsManager.create("Vision No Result", Alert.AlertType.kInfo);
    private double previousVisionResultTimeStamp = 0;
    public double visionObservationRate = 0.0;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer;
    private final Matrix<N3, N1> primaryEstimatorOdometryStdDevs;
    private final Matrix<N3, N1> visionSensitiveEstimatorOdometryStdDevs;

    // Odometry
    private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };
    // Assume gyro starts at zero
    private Rotation2d gyroOffset = new Rotation2d();
    private Pose2d odometryPoseSensorLess = new Pose2d();
    private Pose2d primaryEstimatorPose = new Pose2d();
    private Pose2d visionSensitivePose = new Pose2d();
    private ChassisSpeeds measuredSpeedsRobotRelative = new ChassisSpeeds();

    private boolean lowSpeedModeEnabled = false;

    public enum NavigationMode {
        SENSOR_LESS_ODOMETRY,
        VISION_FUSED_ODOMETRY,
        VISION_GUIDED
    }

    private NavigationMode navigationMode = NavigationMode.VISION_FUSED_ODOMETRY;
    private static final Subsystem lock = new Subsystem() {};

    private RobotState() {
        this.poseBuffer = TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_DURATION.in(Seconds));

        this.primaryEstimatorOdometryStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
        primaryEstimatorOdometryStdDevs.set(0, 0, PRIMARY_ESTIMATOR_ODOMETRY_TRANSLATIONAL_STANDARD_ERROR.in(Meters));
        primaryEstimatorOdometryStdDevs.set(1, 0, PRIMARY_ESTIMATOR_ODOMETRY_TRANSLATIONAL_STANDARD_ERROR.in(Meters));
        primaryEstimatorOdometryStdDevs.set(2, 0, PRIMARY_ESTIMATOR_GYRO_ROTATIONAL_STANDARD_ERROR.in(Radians));

        this.visionSensitiveEstimatorOdometryStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
        visionSensitiveEstimatorOdometryStdDevs.set(
                0, 0, VISION_SENSITIVE_ESTIMATOR_ODOMETRY_TRANSLATIONAL_STANDARD_ERROR.in(Meters));
        visionSensitiveEstimatorOdometryStdDevs.set(
                1, 0, VISION_SENSITIVE_ESTIMATOR_ODOMETRY_TRANSLATIONAL_STANDARD_ERROR.in(Meters));
        visionSensitiveEstimatorOdometryStdDevs.set(
                2, 0, VISION_SENSITIVE_ESTIMATOR_GYRO_ROTATIONAL_STANDARD_ERROR.in(Radians));
    }

    public void resetPose(Pose2d pose) {
        // Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the new
        // frame of rotation
        gyroOffset =
                pose.getRotation().minus(odometryPoseSensorLess.getRotation().minus(gyroOffset));
        primaryEstimatorPose = visionSensitivePose = odometryPoseSensorLess = pose;
        poseBuffer.clear();
    }

    public void addChassisSpeedsObservation(
            SwerveModuleState[] measuredModuleStates, OptionalDouble gyroYawVelocityRadPerSec) {
        ChassisSpeeds wheelSpeeds = DRIVE_KINEMATICS.toChassisSpeeds(measuredModuleStates);
        double angularVelocityRadPerSec = gyroYawVelocityRadPerSec.orElse(wheelSpeeds.omegaRadiansPerSecond);
        this.measuredSpeedsRobotRelative = new ChassisSpeeds(
                wheelSpeeds.vxMetersPerSecond, wheelSpeeds.vyMetersPerSecond, angularVelocityRadPerSec);
    }

    public void addOdometryObservation(OdometryObservation observation) {
        Twist2d twist = DRIVE_KINEMATICS.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        Pose2d lastOdometryPose = odometryPoseSensorLess;
        odometryPoseSensorLess = odometryPoseSensorLess.exp(twist);
        // Use gyro if connected
        observation.gyroAngle.ifPresent(gyroAngle -> {
            // Add offset to measured angle
            Rotation2d angle = gyroAngle.plus(gyroOffset);
            odometryPoseSensorLess = new Pose2d(odometryPoseSensorLess.getTranslation(), angle);
        });
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPoseSensorLess);
        // Calculate diff from last odometry pose and add onto pose estimate
        Twist2d finalTwist = lastOdometryPose.log(odometryPoseSensorLess);
        primaryEstimatorPose = primaryEstimatorPose.exp(finalTwist);
        visionSensitivePose = visionSensitivePose.exp(finalTwist);
    }

    public void addVisionObservation(MapleMultiTagPoseEstimator.VisionObservation observation) {
        // If measurement is old enough to be outside the pose buffer's time-span, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - POSE_BUFFER_DURATION.in(Seconds) > observation.timestamp())
                return;
        } catch (NoSuchElementException ex) {
            return;
        }

        // Get odometry based pose at timestamp
        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) return;

        primaryEstimatorPose = addVisionObservationToEstimator(
                observation, sample.get(), primaryEstimatorPose, primaryEstimatorOdometryStdDevs);
        visionSensitivePose = addVisionObservationToEstimator(
                observation, sample.get(), visionSensitivePose, visionSensitiveEstimatorOdometryStdDevs);

        previousVisionResultTimeStamp = Timer.getTimestamp();
    }

    private Pose2d addVisionObservationToEstimator(
            MapleMultiTagPoseEstimator.VisionObservation observation,
            Pose2d odometryPoseSample,
            Pose2d estimatorPose,
            Matrix<N3, N1> estimatorOdometryStdDevs) {
        // sample --> odometryPose transform and backwards of that
        var sampleToOdometryTransform = new Transform2d(odometryPoseSample, odometryPoseSensorLess);
        var odometryToSampleTransform = new Transform2d(odometryPoseSensorLess, odometryPoseSample);
        // get old estimate by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatorPose.plus(odometryToSampleTransform);

        // Calculate 3 x 3 vision matrix
        var r = new double[3];
        for (int i = 0; i < 3; ++i)
            r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double stdDev = estimatorOdometryStdDevs.get(row, 0);
            if (stdDev == 0.0) visionK.set(row, row, 0.0);
            else visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
        }
        // difference between estimate and vision pose
        Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
        // scale transform by visionK
        var kTimesTransform = visionK.times(VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        Transform2d scaledTransform = new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

        // Recalculate current estimate by applying scaled transform to old estimate
        // then replaying odometry data
        return estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    public Rotation2d getRotation() {
        return primaryEstimatorPose.getRotation();
    }

    public Pose2d getSensorLessOdometryPose() {
        return this.odometryPoseSensorLess;
    }

    public Pose2d getPrimaryEstimatorPose() {
        return primaryEstimatorPose;
    }

    public Pose2d getVisionPose() {
        return this.visionSensitivePose;
    }

    public Pose2d getPose() {
        return switch (navigationMode) {
            case SENSOR_LESS_ODOMETRY -> getSensorLessOdometryPose();
            case VISION_FUSED_ODOMETRY -> getPrimaryEstimatorPose();
            case VISION_GUIDED -> getVisionPose();
        };
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return measuredSpeedsRobotRelative;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), primaryEstimatorPose.getRotation());
    }

    public Pose2d getPoseWithLookAhead() {
        Pose2d currentPose = getPose();
        ChassisSpeeds speeds = getRobotRelativeSpeeds();
        double translationalLookaheadTimeSeconds =
                switch (navigationMode) {
                    case SENSOR_LESS_ODOMETRY -> TRANSLATIONAL_LOOKAHEAD_TIME_SENSOR_LESS.in(Seconds);
                    case VISION_FUSED_ODOMETRY, VISION_GUIDED -> TRANSLATIONAL_LOOKAHEAD_TIME_VISION.in(Seconds);
                };
        double rotationalLookaheadTimeSeconds =
                switch (navigationMode) {
                    case SENSOR_LESS_ODOMETRY -> ROTATIONAL_LOOKAHEAD_TIME_SENSOR_LESS.in(Seconds);
                    case VISION_FUSED_ODOMETRY, VISION_GUIDED -> ROTATIONAL_LOOKAHEAD_TIME_VISION.in(Seconds);
                };
        Twist2d lookAhead = new Twist2d(
                speeds.vxMetersPerSecond * translationalLookaheadTimeSeconds,
                speeds.vyMetersPerSecond * translationalLookaheadTimeSeconds,
                speeds.omegaRadiansPerSecond * rotationalLookaheadTimeSeconds);

        return currentPose.exp(lookAhead);
    }

    public void mergeVisionOdometry() {
        resetPose(visionSensitivePose);
    }

    public Command withNavigationMode(NavigationMode mode) {
        return lock.startEnd(
                () -> {
                    if (mode != NavigationMode.VISION_GUIDED) mergeVisionOdometry();
                    this.navigationMode = mode;
                },
                () -> this.navigationMode = NavigationMode.VISION_FUSED_ODOMETRY);
    }

    public void setLowSpeedMode(boolean lowSpeedModeEnabled) {
        this.lowSpeedModeEnabled = lowSpeedModeEnabled;
    }

    public boolean lowSpeedModeEnabled() {
        return lowSpeedModeEnabled;
    }

    public void updateAlerts() {
        double timeNotVisionResultSeconds = Timer.getTimestamp() - previousVisionResultTimeStamp;
        visionNoResultAlert.set(timeNotVisionResultSeconds > 10);
        if (visionNoResultAlert.get())
            visionNoResultAlert.setText(
                    String.format("No vision pose estimation for %.2f Seconds", timeNotVisionResultSeconds));
        Logger.recordOutput("Vision Observation Rate", visionObservationRate);
    }

    public double visionObservationRate() {
        return visionObservationRate;
    }

    public record OdometryObservation(
            SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }
}

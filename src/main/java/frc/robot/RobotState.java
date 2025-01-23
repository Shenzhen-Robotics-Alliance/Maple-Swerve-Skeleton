package frc.robot;

// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveTrainConstants.*;
import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.apriltags.MapleMultiTagPoseEstimator;
import java.util.NoSuchElementException;
import java.util.Optional;

public class RobotState {
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer;
    private final Matrix<N3, N1> qStdDevs;

    // Odometry
    private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };
    // Assume gyro starts at zero
    private Rotation2d gyroOffset = new Rotation2d();
    private Pose2d odometryPose = new Pose2d();
    private Pose2d estimatedPose = new Pose2d();

    private RobotState() {
        this.qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
        this.poseBuffer = TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_DURATION.in(Seconds));
        qStdDevs.set(0, 0, ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS);
        qStdDevs.set(1, 0, ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS);
        qStdDevs.set(2, 0, GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS);
    }

    public void resetPose(Pose2d pose) {
        // Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the new
        // frame of rotation
        gyroOffset = pose.getRotation().minus(estimatedPose.getRotation().minus(gyroOffset));
        estimatedPose = pose;
        odometryPose = pose;
        poseBuffer.clear();
    }

    public void addOdometryObservation(OdometryObservation observation) {
        Twist2d twist = DRIVE_KINEMATICS.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        Pose2d lastOdometryPose = odometryPose;
        odometryPose = odometryPose.exp(twist);
        // Use gyro if connected
        observation.gyroAngle.ifPresent(gyroAngle -> {
            // Add offset to measured angle
            Rotation2d angle = gyroAngle.plus(gyroOffset);
            odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        });
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);
        // Calculate diff from last odometry pose and add onto pose estimate
        Twist2d finalTwist = lastOdometryPose.log(odometryPose);
        estimatedPose = estimatedPose.exp(finalTwist);
    }

    public void addVisionObservation(MapleMultiTagPoseEstimator.VisionObservation observation) {
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - POSE_BUFFER_DURATION.in(Seconds) > observation.timestamp())
                return;
        } catch (NoSuchElementException ex) {
            return;
        }
        // Get odometry based pose at timestamp
        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) return;

        // sample --> odometryPose transform and backwards of that
        var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        // get old estimate by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Calculate 3 x 3 vision matrix
        var r = new double[3];
        for (int i = 0; i < 3; ++i)
            r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double stdDev = qStdDevs.get(row, 0);
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
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    public Rotation2d getRotation() {
        return estimatedPose.getRotation();
    }

    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public record OdometryObservation(
            SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }
}

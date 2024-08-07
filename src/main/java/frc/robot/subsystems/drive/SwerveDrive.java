// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.subsystems.drive.IO.*;
import frc.robot.utils.Alert;
import frc.robot.utils.Config.MapleConfigFile;

import frc.robot.utils.MapleTimeUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.VisionConfigs.*;

public class SwerveDrive extends MapleSubsystem implements HolonomicDriveSubsystem {
    public final double maxModuleVelocityMetersPerSec, maxAngularVelocityRadPerSec, maxLinearAccelerationMetersPerSecSq;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final OdometryThreadInputsAutoLogged odometryThreadInputs;
    private final SwerveModule[] swerveModules;

    private final Translation2d[] MODULE_TRANSLATIONS;
    public final SwerveDriveKinematics kinematics;
    private Rotation2d rawGyroRotation;
    private final SwerveModulePosition[] lastModulePositions;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final OdometryThread odometryThread;
    private final Alert gyroDisconnectedAlert = new Alert("Gyro Hardware Fault", Alert.AlertType.ERROR),  visionNoResultAlert = new Alert("Vision No Result", Alert.AlertType.INFO);
    public SwerveDrive(GyroIO gyroIO, ModuleIO frontLeftModuleIO, ModuleIO frontRightModuleIO, ModuleIO backLeftModuleIO, ModuleIO backRightModuleIO, MapleConfigFile.ConfigBlock generalConfigBlock) {
        super("Drive");
        this.gyroIO = gyroIO;
        this.rawGyroRotation = new Rotation2d();
        this.swerveModules = new SwerveModule[] {
                new SwerveModule(frontLeftModuleIO, "FrontLeft"),
                new SwerveModule(frontRightModuleIO, "FrontRight"),
                new SwerveModule(backLeftModuleIO, "BackLeft"),
                new SwerveModule(backRightModuleIO, "BackRight"),
        };

        final double horizontalWheelsMarginMeters = generalConfigBlock.getDoubleConfig("horizontalWheelsMarginMeters"),
                verticalWheelsMarginMeters = generalConfigBlock.getDoubleConfig("verticalWheelsMarginMeters"),
                driveBaseRadius = Math.hypot(horizontalWheelsMarginMeters/2, verticalWheelsMarginMeters/2);

        this.maxModuleVelocityMetersPerSec = generalConfigBlock.getDoubleConfig("maxVelocityMetersPerSecond");
        this.maxAngularVelocityRadPerSec = generalConfigBlock.getDoubleConfig("maxAngularVelocityRadiansPerSecond");
        this.maxLinearAccelerationMetersPerSecSq = generalConfigBlock.getDoubleConfig("maxAccelerationMetersPerSecondSquared");
        this.MODULE_TRANSLATIONS = new Translation2d[] {
                new Translation2d(horizontalWheelsMarginMeters / 2, verticalWheelsMarginMeters / 2), // FL
                new Translation2d(horizontalWheelsMarginMeters / 2, -verticalWheelsMarginMeters / 2), // FR
                new Translation2d(-horizontalWheelsMarginMeters / 2, verticalWheelsMarginMeters / 2), // BL
                new Translation2d(-horizontalWheelsMarginMeters / 2, -verticalWheelsMarginMeters / 2)  // BR
        };
        kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
        lastModulePositions = new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, rawGyroRotation, lastModulePositions, new Pose2d(),
                VecBuilder.fill(TRANSLATIONAL_STANDARD_ERROR_METERS_ODOMETRY, TRANSLATIONAL_STANDARD_ERROR_METERS_ODOMETRY, ROTATIONAL_STANDARD_ERROR_RADIANS_ODOMETRY),
                VecBuilder.fill(TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION, TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION, ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION)
        );

        configHolonomicPathPlannerAutoBuilder(driveBaseRadius);

        this.odometryThread = OdometryThread.createInstance();
        this.odometryThreadInputs = new OdometryThreadInputsAutoLogged();
        this.odometryThread.start();

        gyroDisconnectedAlert.setActivated(false);
        visionNoResultAlert.setActivated(false);
    }

    @Override
    public void onReset() {

    }

    @Override
    public void periodic(double dt, boolean enabled) {
        final double t0 = MapleTimeUtils.getRealTimeSeconds();
        fetchOdometryInputs();
        Logger.recordOutput("SystemPerformance/OdometryFetchingTimeMS", (MapleTimeUtils.getRealTimeSeconds() - t0)*1000);
        modulesPeriodic(dt, enabled);

        for (int timeStampIndex = 0; timeStampIndex < odometryThreadInputs.measurementTimeStamps.length; timeStampIndex++)
            feedSingleOdometryDataToPositionEstimator(timeStampIndex);

        final double timeNotVisionResultSeconds = MapleTimeUtils.getLogTimeSeconds() - previousMeasurementTimeStamp;
        visionNoResultAlert.setText(String.format("AprilTag Vision No Result For %.2f (s)", timeNotVisionResultSeconds));
        visionNoResultAlert.setActivated(timeNotVisionResultSeconds > 4);
    }

    private void fetchOdometryInputs() {
        odometryThread.lockOdometry();
        odometryThread.updateInputs(odometryThreadInputs);
        Logger.processInputs("Drive/OdometryThread", odometryThreadInputs);

        for (var module : swerveModules)
            module.updateOdometryInputs();

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        gyroDisconnectedAlert.setActivated(!gyroInputs.connected);

        odometryThread.unlockOdometry();
    }

    private void modulesPeriodic(double dt, boolean enabled) {
        for (var module : swerveModules)
            module.periodic(dt, enabled);
    }

    private void feedSingleOdometryDataToPositionEstimator(int timeStampIndex) {
        final SwerveModulePosition[] modulePositions = getModulesPosition(timeStampIndex),
                moduleDeltas = getModulesDelta(modulePositions);

        if (!updateRobotFacingWithGyroReading(timeStampIndex))
            updateRobotFacingWithOdometry(moduleDeltas);

        poseEstimator.updateWithTime(
                odometryThreadInputs.measurementTimeStamps[timeStampIndex],
                rawGyroRotation,
                modulePositions
        );
    }

    private SwerveModulePosition[] getModulesPosition(int timeStampIndex) {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++)
            swerveModulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[timeStampIndex];
        return swerveModulePositions;
    }

    private SwerveModulePosition[] getModulesDelta(SwerveModulePosition[] freshModulesPosition) {
        SwerveModulePosition[] deltas = new SwerveModulePosition[swerveModules.length];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            final double deltaDistanceMeters = freshModulesPosition[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters;
            deltas[moduleIndex] = new SwerveModulePosition(deltaDistanceMeters, freshModulesPosition[moduleIndex].angle);
            lastModulePositions[moduleIndex] = freshModulesPosition[moduleIndex];
        }
        return deltas;
    }

    /**
     * updates the robot facing using the reading from the gyro
     * @param timeStampIndex the index of the time stamp
     * @return whether the update is success
     * */
    private boolean updateRobotFacingWithGyroReading(int timeStampIndex) {
        if (!gyroInputs.connected)
            return false;
        rawGyroRotation = gyroInputs.odometryYawPositions[timeStampIndex];
        return true;
    }

    /**
     * updates the robot facing using the reading from the gyro
     * @param modulesDelta the delta of the swerve modules calculated from the odometry
     * */
    private void updateRobotFacingWithOdometry(SwerveModulePosition[] modulesDelta) {
        Twist2d twist = kinematics.toTwist2d(modulesDelta);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    @Override
    public void runRawChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxModuleVelocityMetersPerSec);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
            optimizedSetpointStates[i] = swerveModules[i].runSetPoint(setpointStates[i]);

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    @Override
    public void stop() {
        Rotation2d[] swerveHeadings = new Rotation2d[swerveModules.length];
        for (int i = 0; i < swerveHeadings.length; i++)
            swerveHeadings[i] = new Rotation2d();
        kinematics.resetHeadings(swerveHeadings);
        HolonomicDriveSubsystem.super.stop();
    }

    /**
     * Locks the chassis and turns the modules to an X formation to resist movement.
     * The lock will be cancelled the next time a nonzero velocity is requested.
     */
    public void lockChassisWithXFormation() {
        Rotation2d[] swerveHeadings = new Rotation2d[swerveModules.length];
        for (int i = 0; i < swerveHeadings.length; i++)
            swerveHeadings[i] = MODULE_TRANSLATIONS[i].getAngle();
        kinematics.resetHeadings(swerveHeadings);
        HolonomicDriveSubsystem.super.stop();
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < states.length; i++)
            states[i] = swerveModules[i].getMeasuredState();
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all the modules.
     */
    private SwerveModulePosition[] getModuleLatestPositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < states.length; i++)
            states[i] = swerveModules[i].getLatestPosition();
        return states;
    }

    @AutoLogOutput(key="Odometry/RobotPosition")
    @Override
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    @Override
    public Rotation2d getRawGyroYaw() {return gyroInputs.yawPosition; }

    @Override
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModuleLatestPositions(), pose);
    }

    @Override
    public ChassisSpeeds getMeasuredChassisSpeedsRobotRelative() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    @Override public double getChassisMaxLinearVelocityMetersPerSec() {return maxModuleVelocityMetersPerSec;}
    @Override public double getChassisMaxAccelerationMetersPerSecSq() {return maxLinearAccelerationMetersPerSecSq;}
    @Override public double getChassisMaxAngularVelocity() {return maxAngularVelocityRadPerSec;}
    @Override public double getChassisMaxAngularAccelerationRadPerSecSq() {return Constants.ChassisDefaultConfigs.DEFAULT_MAX_ANGULAR_ACCELERATION_DEGREES_PER_SECOND_SQUARE;}

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> measurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp ,measurementStdDevs);
        previousMeasurementTimeStamp = Math.max(timestamp, previousMeasurementTimeStamp);
    }

    private double previousMeasurementTimeStamp = -1;
    @Override
    public double getPreviousVisionMeasurementTimeStamp() {
        return previousMeasurementTimeStamp;
    }
}

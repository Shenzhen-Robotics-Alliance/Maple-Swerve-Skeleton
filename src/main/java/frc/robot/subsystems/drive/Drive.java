// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.LocalADStarAK;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends MapleSubsystem {
    private final double maxModuleVelocityMetersPerSec, maxAngularVelocityRadPerSec;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final OdometryThreadInputsAutoLogged odometryThreadInputs;
    private final SwerveModule[] swerveModules;

    private final SwerveDriveKinematics kinematics;
    private Rotation2d rawGyroRotation;
    private final SwerveModulePosition[] lastModulePositions;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final OdometryThread odometryThread;
    public Drive(GyroIO gyroIO, ModuleIO frontLeftModuleIO, ModuleIO frontRightModuleIO, ModuleIO backLeftModuleIO, ModuleIO backRightModuleIO, MapleConfigFile.ConfigBlock generalConfigBlock) {
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
        final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
                new Translation2d(horizontalWheelsMarginMeters / 2, verticalWheelsMarginMeters / 2), // FL
                new Translation2d(horizontalWheelsMarginMeters / 2, -verticalWheelsMarginMeters / 2), // FR
                new Translation2d(-horizontalWheelsMarginMeters / 2, verticalWheelsMarginMeters / 2), // BL
                new Translation2d(-horizontalWheelsMarginMeters / 2, -verticalWheelsMarginMeters / 2)  // BR
        };
        kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
        lastModulePositions = new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

        this.odometryThread = OdometryThread.createInstance();
        this.odometryThreadInputs = new OdometryThreadInputsAutoLogged();
        this.odometryThread.start();


        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity,
                new HolonomicPathFollowerConfig(maxModuleVelocityMetersPerSec, driveBaseRadius, new ReplanningConfig()),
                () -> DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red),
                this
        );
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]))
        );
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
        );
    }

    @Override
    public void onReset() {

    }

    @Override
    public void periodic(double dt, boolean enabled) {
        fetchOdometryInputs();
        modulesPeriodic(dt, enabled);
        feedOdometryDataToPositionEstimator();
    }

    private void fetchOdometryInputs() {
        long nanos = System.nanoTime();

        odometryThread.lockOdometry();
        odometryThread.updateInputs(odometryThreadInputs);
        Logger.processInputs("Drive/OdometryThread", odometryThreadInputs);

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (var module : swerveModules)
            module.fetchOdometryInputs();

        odometryThread.unlockOdometry();
        Logger.recordOutput(Constants.LogConfigs.SYSTEM_PERFORMANCE_PATH + "Drive/OdometryFetchCPUTimeMS", (System.nanoTime()-nanos) * 0.000001);
    }

    private void modulesPeriodic(double dt, boolean enabled) {
        for (var module : swerveModules)
            module.periodic(dt, enabled);
    }

    private void feedOdometryDataToPositionEstimator() { // TODO make this look prettier
        Logger.recordOutput("/Odometry/timeStampsLength", odometryThreadInputs.measurementTimeStamps.length);
        for (int i = 0; i < odometryThreadInputs.measurementTimeStamps.length; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                        new SwerveModulePosition(
                                modulePositions[moduleIndex].distanceMeters
                                        - lastModulePositions[moduleIndex].distanceMeters,
                                modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(odometryThreadInputs.measurementTimeStamps[i], rawGyroRotation, modulePositions);
        }
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxModuleVelocityMetersPerSec);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = swerveModules[i].requestSetPoint(setpointStates[i]);
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void requestStopWithX() {
        for (SwerveModule swerveModule:swerveModules)
            swerveModule.requestXFormationSetpoint();
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
            states[i] = swerveModules[i].getMeasuredState();
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++)
            states[i] = swerveModules[i].getLatestPosition();
        return states;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current odometry pose.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp  The timestamp of the vision measurement in seconds.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }
}

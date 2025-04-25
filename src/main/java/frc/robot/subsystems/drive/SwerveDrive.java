// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveTrainConstants.*;

import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.commands.SwerveSubsystem;
import frc.robot.subsystems.drive.IO.*;
import frc.robot.utils.AlertsManager;
import frc.robot.utils.MapleTimeUtils;
import frc.robot.utils.TipOverDetection;
import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalDouble;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase implements SwerveSubsystem {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs;
    private final OdometryThreadInputsAutoLogged odometryThreadInputs;
    private final CanBusIO canBusIO;
    private final CanBusIO.CanBusInputs canBusInputs;
    private final SwerveModule[] swerveModules;

    private final OdometryThread odometryThread;

    // Alerts
    private final Alert gyroDisconnectedAlert =
            AlertsManager.create("Gyro hardware fault detected!", Alert.AlertType.kError);
    private final Alert gyroConfigurationFailed = AlertsManager.create(
            "Gyro configuration failed! Reboot robot after fixing connection.", Alert.AlertType.kError);
    private final Alert canBusHighUtilization =
            AlertsManager.create("Drivetrain CanBus high utilization!", Alert.AlertType.kError);
    private final Alert batteryBrownoutAlert =
            AlertsManager.create("Battery brownout detected!", Alert.AlertType.kError);
    private final Debouncer drivetrainOverCurrentDebouncer = new Debouncer(0.2, Debouncer.DebounceType.kBoth);
    private final Alert drivetrainOverCurrentAlert =
            AlertsManager.create("Drivetrain over current detected! Current: ", Alert.AlertType.kError);

    private final Debouncer batteryBrownoutDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer robotTippingDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);

    private boolean robotTipping = false;
    private static final Angle TIP_OVER_THRESHOLD = Degrees.of(2.4);
    public Trigger driveTrainTipping = new Trigger(() -> robotTipping);

    public SwerveDrive(
            OdometryThread odometryThread,
            GyroIO gyroIO,
            CanBusIO canBusIO,
            ModuleIO frontLeftModuleIO,
            ModuleIO frontRightModuleIO,
            ModuleIO backLeftModuleIO,
            ModuleIO backRightModuleIO) {
        this.odometryThread = odometryThread;
        this.gyroIO = gyroIO;
        this.canBusIO = canBusIO;
        this.canBusInputs = new CanBusIO.CanBusInputs();
        this.gyroInputs = new GyroIOInputsAutoLogged();
        this.swerveModules = new SwerveModule[] {
            new SwerveModule(frontLeftModuleIO, "FrontLeft"),
            new SwerveModule(frontRightModuleIO, "FrontRight"),
            new SwerveModule(backLeftModuleIO, "BackLeft"),
            new SwerveModule(backRightModuleIO, "BackRight"),
        };

        this.odometryThreadInputs = new OdometryThreadInputsAutoLogged();
        this.odometryThread.start();

        gyroDisconnectedAlert.set(false);
        batteryBrownoutAlert.set(false);
        drivetrainOverCurrentAlert.set(false);
        // Prevents fake alerts to show up dues to falling type debounce
        MapleTimeUtils.delay(0.5);

        startDashboardDisplay();
    }

    @Override
    public void periodic() {
        fetchOdometryInputs();
        modulesPeriodic();

        for (int timeStampIndex = 0;
                timeStampIndex < odometryThreadInputs.odometryTicksCountInPreviousRobotPeriod;
                timeStampIndex++) feedSingleOdometryDataToPositionEstimator(timeStampIndex);

        RobotState.getInstance()
                .addChassisSpeedsObservation(
                        getModuleStates(),
                        gyroInputs.connected
                                ? OptionalDouble.of(gyroInputs.yawVelocityRadPerSec)
                                : OptionalDouble.empty());

        RobotState.getInstance().updateAlerts();
        gyroConfigurationFailed.set(gyroInputs.configurationFailed);
        gyroDisconnectedAlert.set(!gyroInputs.configurationFailed && !gyroInputs.connected);
        canBusHighUtilization.setText(
                "Drivetrain CanBus high utilization: " + (int) (canBusInputs.utilization * 100) + "%");
        canBusHighUtilization.set(canBusInputs.utilization > 0.8);
        batteryBrownoutAlert.set(batteryBrownoutDebouncer.calculate(RobotController.isBrownedOut()));
        drivetrainOverCurrentAlert.set(drivetrainOverCurrentDebouncer.calculate(
                getDriveTrainTotalCurrentAmps() > OVER_CURRENT_WARNING.in(Amps)));

        Logger.recordOutput(
                "RobotState/SensorLessOdometryPose", RobotState.getInstance().getSensorLessOdometryPose());
        Logger.recordOutput(
                "RobotState/PrimaryEstimatorPose", RobotState.getInstance().getPrimaryEstimatorPose());
        Logger.recordOutput(
                "RobotState/PrimaryEstimatorPoseWith3dRot",
                new Pose3d(
                        new Translation3d(RobotState.getInstance()
                                .getPrimaryEstimatorPose()
                                .getTranslation()),
                        getDriveTrain3dOrientation()));
        Logger.recordOutput(
                "RobotState/VisionSensitivePose", RobotState.getInstance().getVisionPose());
        Logger.recordOutput(
                "RobotState/ControlLoopPose", RobotState.getInstance().getPose());
        Logger.recordOutput(
                "RobotState/ControlLoopPoseWithLookAhead",
                RobotState.getInstance().getPoseWithLookAhead());

        robotTipping = robotTippingDebouncer.calculate(
                TipOverDetection.getTippingAngleRad(getDriveTrain3dOrientation()) > TIP_OVER_THRESHOLD.in(Radians));
        Logger.recordOutput(
                "RobotState/TippingAngleDeg",
                Math.toDegrees(TipOverDetection.getTippingAngleRad(getDriveTrain3dOrientation())));
    }

    private void fetchOdometryInputs() {
        odometryThread.lockOdometry();
        odometryThread.updateInputs(odometryThreadInputs);
        Logger.processInputs("Drive/OdometryThread", odometryThreadInputs);

        for (var module : swerveModules) module.updateOdometryInputs();

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        canBusIO.updateInputs(canBusInputs);
        Logger.processInputs("Drive/CANBus", canBusInputs);

        odometryThread.unlockOdometry();
    }

    private void modulesPeriodic() {
        for (var module : swerveModules) module.modulePeriodic();
    }

    private void feedSingleOdometryDataToPositionEstimator(int timeStampIndex) {
        final SwerveModulePosition[] modulePositions = getModulesPosition(timeStampIndex);

        RobotState.getInstance()
                .addOdometryObservation(new RobotState.OdometryObservation(
                        modulePositions,
                        gyroInputs.connected
                                ? Optional.of(gyroInputs.odometryYawPositions[timeStampIndex])
                                : Optional.empty(),
                        odometryThreadInputs.measurementTimeStamps[timeStampIndex]));
    }

    private SwerveModulePosition[] getModulesPosition(int timeStampIndex) {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++)
            swerveModulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[timeStampIndex];
        return swerveModulePositions;
    }

    @Override
    public void executeSetpoint(SwerveSetpoint setpoint) {
        if (Robot.LOG_DETAILS)
            Logger.recordOutput("SwerveSetpoint/currentSwerveSpeeds", setpoint.robotRelativeSpeeds());
        ChassisSpeeds speeds = setpoint.robotRelativeSpeeds();

        if (Robot.LOG_DETAILS) Logger.recordOutput("SwerveSetpoint/executedSpeeds", speeds);

        SwerveModuleState[] setPointStates = DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setPointStates, CHASSIS_MAX_VELOCITY);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
            optimizedSetpointStates[i] = swerveModules[i].runSetPoint(
                    setPointStates[i],
                    setpoint.feedforwards().robotRelativeForcesX()[i],
                    setpoint.feedforwards().robotRelativeForcesY()[i]);

        Logger.recordOutput("SwerveStates/Setpoints", setPointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    @Override
    public void stop() {
        Rotation2d[] swerveHeadings = new Rotation2d[swerveModules.length];
        for (int i = 0; i < swerveHeadings.length; i++) swerveHeadings[i] = new Rotation2d();
        DRIVE_KINEMATICS.resetHeadings(swerveHeadings);
        for (SwerveModule module : swerveModules) module.stop();
    }

    /** Turns the motor brakes on */
    public void setMotorBrake(boolean motorBrakeEnabled) {
        for (SwerveModule module : swerveModules) module.setMotorBrake(motorBrakeEnabled);
    }

    /** Returns the module states (turn angles and drive velocities) for all the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < states.length; i++) states[i] = swerveModules[i].getMeasuredState();
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all the modules. */
    private SwerveModulePosition[] getModuleLatestPositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < states.length; i++) states[i] = swerveModules[i].getLatestPosition();
        return states;
    }

    private void startDashboardDisplay() {
        SmartDashboard.putData("Swerve Drive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                    "Front Left Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
            builder.addDoubleProperty(
                    "Front Left Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                    "Front Right Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
            builder.addDoubleProperty(
                    "Front Right Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                    "Back Left Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
            builder.addDoubleProperty(
                    "Back Left Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                    "Back Right Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
            builder.addDoubleProperty(
                    "Back Right Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                    "Robot Angle",
                    () -> RobotState.getInstance()
                            .getPose()
                            .getRotation()
                            .minus(FieldMirroringUtils.getCurrentAllianceDriverStationFacing())
                            .getRadians(),
                    null);
        });
    }

    public double getCanBusUtilization() {
        return canBusInputs.utilization;
    }

    public Rotation3d getDriveTrain3dOrientation() {
        return new Rotation3d(
                gyroInputs.rollRad,
                gyroInputs.pitchRad,
                RobotState.getInstance().getPose().getRotation().getRadians());
    }

    @AutoLogOutput(key = "DrivetrainTotalCurrentAmps")
    public double getDriveTrainTotalCurrentAmps() {
        return Arrays.stream(swerveModules)
                .mapToDouble(SwerveModule::getTotalSupplyCurrentAmps)
                .sum();
    }

    @Override
    public void runCharacterization(Voltage voltage) {
        SwerveModuleState[] moduleStates = DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, 0, 1));
        for (int i = 0; i < swerveModules.length; i++)
            swerveModules[i].runVoltageCharacterization(moduleStates[i].angle, voltage.in(Volts));
    }
}

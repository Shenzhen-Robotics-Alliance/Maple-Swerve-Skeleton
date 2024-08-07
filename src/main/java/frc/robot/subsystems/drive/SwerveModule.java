// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.subsystems.drive.IO.ModuleIO;
import frc.robot.subsystems.drive.IO.ModuleIOInputsAutoLogged;
import frc.robot.utils.Alert;
import frc.robot.utils.MapleMaths.SwerveStateProjection;
import frc.robot.utils.MechanismControl.InterpolatedMotorFeedForward;
import frc.robot.utils.MechanismControl.MaplePIDController;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends MapleSubsystem {
    private final ModuleIO io;
    private final String name;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private final InterpolatedMotorFeedForward driveOpenLoop;
    private final PIDController turnCloseLoop;
    private SwerveModuleState setPoint;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[]{};

    private final Alert hardwareFaultAlert;

    public SwerveModule(ModuleIO io, String name) {
        super("Module-" + name);
        this.io = io;
        this.name = name;
        this.hardwareFaultAlert = new Alert(
                "Module-" + name + " Hardware Fault",
                Alert.AlertType.ERROR
        );
        this.hardwareFaultAlert.setActivated(false);

        driveOpenLoop = InterpolatedMotorFeedForward.fromDeployedDirectory("DrivingMotorOpenLoop");
        turnCloseLoop = new MaplePIDController(Constants.SwerveModuleConfigs.steerHeadingCloseLoopConfig);

        CommandScheduler.getInstance().unregisterSubsystem(this);

        io.setDriveBrake(true);
        io.setSteerBrake(true);
    }

    @Override
    public void onReset() {
        setPoint = new SwerveModuleState();
        onDisable();
        turnCloseLoop.calculate(getSteerFacing().getRadians()); // activate close loop controller
    }

    public void updateOdometryInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module-" + name, inputs);
        this.hardwareFaultAlert.setActivated(!inputs.hardwareConnected);
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        updateOdometryPositions();
    }

    private void updateOdometryPositions() {
        odometryPositions = new SwerveModulePosition[inputs.odometryDriveWheelRevolutions.length];
        for (int i = 0; i < odometryPositions.length; i++) {
            double positionMeters = toDrivePositionMeters(inputs.odometryDriveWheelRevolutions[i]);
            Rotation2d angle = inputs.odometrySteerPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    private void runSteerCloseLoop() {
        turnCloseLoop.setSetpoint(setPoint.angle.getRadians());
        io.setSteerPowerPercent(turnCloseLoop.calculate(getSteerFacing().getRadians()));
    }

    private void runDriveOpenLoop() {
        final double adjustSpeedSetpointMetersPerSec = SwerveStateProjection.project(setPoint, getSteerFacing());
        Logger.recordOutput("/SwerveStates/FeedForward/" + this.name + "/required velocity", adjustSpeedSetpointMetersPerSec);
        Logger.recordOutput("/SwerveStates/FeedForward/" + this.name + "/corresponding power (mag)", driveOpenLoop.calculate(adjustSpeedSetpointMetersPerSec));
        io.setDriveSpeedPercent(driveOpenLoop.calculate(adjustSpeedSetpointMetersPerSec));
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized state.
     */
    public SwerveModuleState runSetPoint(SwerveModuleState state) {
        this.setPoint = SwerveModuleState.optimize(state, getSteerFacing());

        runDriveOpenLoop();
        runSteerCloseLoop();

        return this.setPoint;
    }

    @Override
    public void onDisable() {
        io.setSteerPowerPercent(0);
        io.setDriveSpeedPercent(0);
    }

    /**
     * Returns the current turn angle of the module.
     */
    public Rotation2d getSteerFacing() {
        return inputs.steerFacing;
    }

    public double getSteerVelocityRadPerSec() {
        return inputs.steerVelocityRadPerSec;
    }

    /**
     * Returns the current drive position of the module in meters.
     */
    public double getDrivePositionMeters() {
        return toDrivePositionMeters(inputs.driveWheelFinalRevolutions);
    }

    private double toDrivePositionMeters(double driveWheelRevolutions) {
        return Units.rotationsToRadians(driveWheelRevolutions) * Constants.SwerveModuleConfigs.WHEEL_RADIUS;
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getDriveVelocityMetersPerSec() {
        return toDrivePositionMeters(inputs.driveWheelFinalVelocityRevolutionsPerSec);
    }

    /**
     * Returns the module position (turn angle and drive position).
     */
    public SwerveModulePosition getLatestPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getSteerFacing());
    }

    /**
     * Returns the module state (turn angle and drive velocity).
     */
    public SwerveModuleState getMeasuredState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSec(), getSteerFacing());
    }

    /**
     * Returns the module positions received this cycle.
     */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }
}

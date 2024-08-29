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
import frc.robot.constants.DriveTrainConstants;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.subsystems.drive.IO.ModuleIO;
import frc.robot.subsystems.drive.IO.ModuleIOInputsAutoLogged;
import frc.robot.utils.Alert;
import frc.robot.utils.CustomMaths.SwerveStateProjection;
import frc.robot.utils.CustomPIDs.MaplePIDController;
import org.littletonrobotics.junction.Logger;

import static frc.robot.constants.DriveControlLoops.*;

public class SwerveModule extends MapleSubsystem {
    private final ModuleIO io;
    private final String name;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private final PIDController turnCloseLoop, driveCloseLoop;
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

        turnCloseLoop = new MaplePIDController(STEER_CLOSE_LOOP);
        driveCloseLoop = new MaplePIDController(DRIVE_CLOSE_LOOP);

        CommandScheduler.getInstance().unregisterSubsystem(this);

        setPoint = new SwerveModuleState();
        turnCloseLoop.calculate(getSteerFacing().getRadians()); // activate close loop controller
        io.setDriveBrake(true);
        io.setSteerBrake(true);
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
            double positionMeters = driveWheelRevolutionsToMeters(inputs.odometryDriveWheelRevolutions[i]);
            Rotation2d angle = inputs.odometrySteerPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    private void runSteerCloseLoop() {
        turnCloseLoop.setSetpoint(setPoint.angle.getRadians());
        io.setSteerPowerPercent(turnCloseLoop.calculate(getSteerFacing().getRadians()));
    }

    private void runDriveControlLoop() {
        final double adjustSpeedSetpointMetersPerSec = SwerveStateProjection.project(setPoint, getSteerFacing());
        io.setDriveVoltage(
                DRIVE_OPEN_LOOP.calculate(adjustSpeedSetpointMetersPerSec)
                + driveCloseLoop.calculate(getDriveVelocityMetersPerSec(), adjustSpeedSetpointMetersPerSec)
        );
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized state.
     */
    public SwerveModuleState runSetPoint(SwerveModuleState state) {
        this.setPoint = SwerveModuleState.optimize(state, getSteerFacing());

        runDriveControlLoop();
        runSteerCloseLoop();

        return this.setPoint;
    }

    @Override
    public void onDisable() {
        io.setSteerPowerPercent(0);
        io.setDriveVoltage(0);
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
        return driveWheelRevolutionsToMeters(inputs.driveWheelFinalRevolutions);
    }

    private double driveWheelRevolutionsToMeters(double driveWheelRevolutions) {
        return Units.rotationsToRadians(driveWheelRevolutions) * DriveTrainConstants.WHEEL_RADIUS_METERS;
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getDriveVelocityMetersPerSec() {
        return driveWheelRevolutionsToMeters(inputs.driveWheelFinalVelocityRevolutionsPerSec);
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

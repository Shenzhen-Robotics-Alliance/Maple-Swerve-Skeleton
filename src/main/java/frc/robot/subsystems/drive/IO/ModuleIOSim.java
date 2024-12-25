// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.DriveTrainConstants;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized to a random value.
 * The flywheel sims are not physically accurate, but provide a decent approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    private static final double DRIVE_KS = 0.03;
    private static final double DRIVE_KP = 0.05;
    private static final double STEER_KP = 8.0;
    private static final double STEER_KD = 0.0;

    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor, steerMotor;

    private boolean driveClosedLoopActivated = false;
    private boolean steerClosedLoopActivated = false;
    private final PIDController driveController;
    private final PIDController steerController;
    private double driveAppliedVolts = 0.0;
    private double steerAppliedVolts = 0.0;
    private double desiredWheelVelocityRadPerSec = 0.0;
    private Rotation2d desiredSteerFacing = new Rotation2d();

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        this.driveMotor = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(DriveTrainConstants.DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT);
        this.steerMotor = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(DriveTrainConstants.STEER_CURRENT_LIMIT);

        this.driveController = new PIDController(DRIVE_KP, 0.0, 0.0);
        this.steerController = new PIDController(STEER_KP, 0.0, STEER_KD);

        // Enable wrapping for turn PID
        steerController.enableContinuousInput(-Math.PI, Math.PI);
        SimulatedArena.getInstance().addCustomSimulation((subTickNum) -> runControlLoops());
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveWheelFinalRevolutions =
                moduleSimulation.getDriveWheelFinalPosition().in(Revolutions);
        inputs.driveWheelFinalVelocityRevolutionsPerSec =
                moduleSimulation.getDriveWheelFinalSpeed().in(RevolutionsPerSecond);
        inputs.driveMotorAppliedVolts =
                moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);
        inputs.driveMotorCurrentAmps =
                moduleSimulation.getDriveMotorStatorCurrent().in(Amps);

        inputs.steerFacing = moduleSimulation.getSteerAbsoluteFacing();
        inputs.steerVelocityRadPerSec =
                moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.steerMotorAppliedVolts =
                moduleSimulation.getSteerMotorAppliedVoltage().in(Volts);
        inputs.steerMotorCurrentAmps =
                moduleSimulation.getSteerMotorStatorCurrent().in(Amps);

        inputs.odometryDriveWheelRevolutions = Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Revolutions))
                .toArray();
        inputs.odometrySteerPositions = moduleSimulation.getCachedSteerAbsolutePositions();
    }

    public void runControlLoops() {
        // Run control loops if activated
        if (driveClosedLoopActivated) calculateDriveControlLoops();
        else driveController.reset();
        if (steerClosedLoopActivated) calculateSteerControlLoops();
        else steerController.reset();

        // Feed voltage to motor simulation
        driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
        steerMotor.requestVoltage(Volts.of(steerAppliedVolts));
    }

    private void calculateDriveControlLoops() {
        double ffVolts = moduleSimulation.driveMotorConfigs.motor.getVoltage(
                        0, desiredWheelVelocityRadPerSec * moduleSimulation.DRIVE_GEAR_RATIO)
                + Math.signum(desiredWheelVelocityRadPerSec) * DRIVE_KS;
        double fbVolts = driveController.calculate(
                moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond), desiredWheelVelocityRadPerSec);
        driveAppliedVolts = ffVolts + fbVolts;
    }

    private void calculateSteerControlLoops() {
        steerAppliedVolts = steerController.calculate(
                moduleSimulation.getSteerAbsoluteFacing().getRadians(), desiredSteerFacing.getRadians());
    }

    @Override
    public void requestDriveOpenLoop(Voltage output) {
        this.driveAppliedVolts = output.in(Volts);
        this.driveClosedLoopActivated = false;
    }

    @Override
    public void requestSteerOpenLoop(Voltage output) {
        this.steerAppliedVolts = output.in(Volts);
        this.steerClosedLoopActivated = false;
    }

    @Override
    public void requestDriveOpenLoop(Current output) {
        DCMotor driveMotorModel = moduleSimulation.driveMotorConfigs.motor;
        this.driveAppliedVolts = driveMotorModel.getVoltage(
                driveMotorModel.getCurrent(output.in(Amps)),
                moduleSimulation.getDriveEncoderUnGearedSpeed().in(RadiansPerSecond));
        this.driveClosedLoopActivated = false;
    }

    @Override
    public void requestSteerOpenLoop(Current output) {
        DCMotor steerMotorModel = moduleSimulation.getSteerMotorConfigs().motor;
        this.driveAppliedVolts = steerMotorModel.getVoltage(
                steerMotorModel.getCurrent(output.in(Amps)),
                moduleSimulation.getDriveEncoderUnGearedSpeed().in(RadiansPerSecond));
        this.steerClosedLoopActivated = false;
    }

    @Override
    public void requestDriveVelocityControl(double desiredWheelVelocityRadPerSec) {
        this.desiredWheelVelocityRadPerSec = desiredWheelVelocityRadPerSec;
        this.driveClosedLoopActivated = true;
    }

    @Override
    public void requestSteerPositionControl(Rotation2d desiredSteerAbsoluteFacing) {
        this.desiredSteerFacing = desiredSteerAbsoluteFacing;
        this.steerClosedLoopActivated = true;
    }
}

// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveTrainConstants.DRIVE_GEAR_RATIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.DriveTrainConstants;
import frc.robot.generated.TunerConstants;
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
    private static final double DRIVE_KP = 0.12;
    private static final double STEER_KP = 8.0;
    private static final double STEER_KD = 0.0;

    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor, steerMotor;

    private boolean driveClosedLoopActivated = false;
    private boolean steerClosedLoopActivated = false;
    private final PIDController driveController;
    private final PIDController steerController;
    private double desiredMotorVelocityRadPerSec = 0.0;
    private double torqueFeedforwardVolts = 0.0;
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
        inputs.hardwareCurrentlyConnected = true;

        inputs.driveMotorAppliedVolts = driveMotor.getAppliedVoltage().in(Volts);
        inputs.steerMotorAppliedVolts = steerMotor.getAppliedVoltage().in(Volts);

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

    }

    private void calculateDriveControlLoops() {
        DCMotor motorModel = moduleSimulation.config.driveMotorConfigs.motor;
        double frictionTorque =
                motorModel.getTorque(motorModel.getCurrent(0, TunerConstants.FrontLeft.DriveFrictionVoltage))
                        * Math.signum(desiredMotorVelocityRadPerSec);
        double velocityFeedforwardVolts = motorModel.getVoltage(frictionTorque, desiredMotorVelocityRadPerSec);
        double feedforwardVolts = velocityFeedforwardVolts + torqueFeedforwardVolts;
        double feedBackVolts = driveController.calculate(
                moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond),
                desiredMotorVelocityRadPerSec / DRIVE_GEAR_RATIO);

        double driveVoltage = feedforwardVolts + feedBackVolts;
        driveMotor.requestVoltage(Volts.of(DriverStation.isEnabled() ? driveVoltage : 0.0));
    }

    private void calculateSteerControlLoops() {
        double steerVoltage = steerController.calculate(
                moduleSimulation.getSteerAbsoluteFacing().getRadians(), desiredSteerFacing.getRadians());
        steerMotor.requestVoltage(Volts.of(DriverStation.isEnabled() ? steerVoltage : 0.0));
    }

    @Override
    public void requestDriveOpenLoop(double outputVolts) {
        this.driveClosedLoopActivated = false;
        driveMotor.requestVoltage(Volts.of(outputVolts));
    }

    @Override
    public void requestSteerOpenLoop(double outputVolts) {
        this.steerClosedLoopActivated = false;
        steerMotor.requestVoltage(Volts.of(outputVolts));
    }

    @Override
    public void requestDriveVelocityControl(double desiredMotorVelocityRadPerSec, double feedforwardMotorVoltage) {
        this.desiredMotorVelocityRadPerSec = desiredMotorVelocityRadPerSec;
        this.driveClosedLoopActivated = true;
        this.torqueFeedforwardVolts = feedforwardMotorVoltage;
    }

    @Override
    public void requestSteerPositionControl(Rotation2d desiredSteerAbsoluteFacing) {
        this.desiredSteerFacing = desiredSteerAbsoluteFacing;
        this.steerClosedLoopActivated = true;
    }
}

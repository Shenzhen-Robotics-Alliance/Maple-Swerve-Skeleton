// By 5516 Iron Maple for maple-sim
// https://github.com/Shenzhen-Robotics-Alliance/maple-sim

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveTrainConstants.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX implements SimulatedArena.Simulatable {
    private final SwerveModuleSimulation simulation;
    private final PIDController steerController;
    private final SimulatedMotorController.GenericMotorController steerMotor;

    private Rotation2d closeLoopSetpoint = new Rotation2d();
    private Voltage openLoopOutput = Volts.zero();
    private boolean isCloseLoop = false;

    public ModuleIOTalonFXSim(SwerveModuleConstants constants, SwerveModuleSimulation simulation) {
        super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

        this.simulation = simulation;
        simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));

        steerMotor = simulation.useSteerMotorController(
                new SimulatedMotorController.GenericMotorController(STEER_MOTOR_MODEL)
                        .withCurrentLimit(STEER_CURRENT_LIMIT));
        this.steerController = new PIDController(10.0, 0, 0.5);
        steerController.enableContinuousInput(-Math.PI, Math.PI);

        SimulatedArena.getInstance().addCustomSimulation(this);
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        double steerOutputVolts = isCloseLoop
                ? steerController.calculate(
                        simulation.getSteerAbsoluteAngle().in(Radians), closeLoopSetpoint.getRadians())
                : openLoopOutput.in(Volts);
        steerMotor.requestVoltage(Volts.of(steerOutputVolts));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        inputs.steerMotorConnected = true;
        inputs.steerEncoderConnected = true;
        inputs.steerFacing = simulation.getSteerAbsoluteFacing();
        inputs.steerVelocityRadPerSec =
                simulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.steerMotorAppliedVolts = steerMotor.getAppliedVoltage().in(Volts);
        inputs.steerMotorCurrentAmps = simulation.getSteerMotorStatorCurrent().in(Amps);

        // Update odometry inputs
        inputs.odometryDriveWheelRevolutions = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Rotations))
                .toArray();

        inputs.odometrySteerPositions = simulation.getCachedSteerAbsolutePositions();
    }

    @Override
    public void requestSteerOpenLoop(Voltage output) {
        openLoopOutput = output;
        isCloseLoop = false;
    }

    @Override
    public void requestSteerPositionControl(Rotation2d desiredSteerAbsoluteFacing) {
        closeLoopSetpoint = desiredSteerAbsoluteFacing;
        isCloseLoop = true;
    }
}

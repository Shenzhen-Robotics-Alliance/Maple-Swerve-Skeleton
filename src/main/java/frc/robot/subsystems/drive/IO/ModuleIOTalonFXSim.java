// By 5516 Iron Maple for maple-sim
// https://github.com/Shenzhen-Robotics-Alliance/maple-sim

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.utils.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
    private final SwerveModuleSimulation simulation;

    public ModuleIOTalonFXSim(SwerveModuleConstants constants, SwerveModuleSimulation simulation) {
        super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

        this.simulation = simulation;
        simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));

        simulation.useSteerMotorController(
                new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(steerTalon, cancoder));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        // Update odometry inputs
        inputs.odometryDriveWheelRevolutions = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Rotations))
                .toArray();

        inputs.odometrySteerPositions = simulation.getCachedSteerAbsolutePositions();
    }
}

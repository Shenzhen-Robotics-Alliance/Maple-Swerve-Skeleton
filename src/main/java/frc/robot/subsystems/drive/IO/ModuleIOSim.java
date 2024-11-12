// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.ControlRequest;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized to a random value.
 * The flywheel sims are not physically accurate, but provide a decent approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveWheelFinalRevolutions = Units.radiansToRotations(moduleSimulation.getDriveWheelFinalPositionRad());
        inputs.driveWheelFinalVelocityRevolutionsPerSec =
                Units.radiansToRotations(moduleSimulation.getDriveWheelFinalSpeedRadPerSec());
        inputs.driveMotorAppliedVolts = moduleSimulation.getDriveMotorAppliedVolts();
        inputs.driveMotorCurrentAmps = moduleSimulation.getDriveMotorSupplyCurrentAmps();

        inputs.steerFacing = moduleSimulation.getSteerAbsoluteFacing();
        inputs.steerVelocityRadPerSec = moduleSimulation.getSteerAbsoluteEncoderSpeedRadPerSec();
        inputs.steerMotorAppliedVolts = moduleSimulation.getSteerMotorAppliedVolts();
        inputs.steerMotorCurrentAmps = moduleSimulation.getSteerMotorSupplyCurrentAmps();

        inputs.odometryDriveWheelRevolutions = Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositionsRad())
                .map(Units::radiansToRotations)
                .toArray();

        inputs.odometrySteerPositions = moduleSimulation.getCachedSteerAbsolutePositions();

        inputs.hardwareConnected = true;
    }

    @Override
    public void setDriveVoltage(double volts) {
        moduleSimulation.requestDriveControl(new ControlRequest.VoltageOut(Volts.of(volts)));
    }

    @Override
    public void setSteerPowerPercent(double powerPercent) {
        moduleSimulation.requestSteerControl(
                new ControlRequest.VoltageOut(Volts.of(12).times(powerPercent)));
    }
}

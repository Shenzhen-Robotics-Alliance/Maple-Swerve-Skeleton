// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Modified by 5516 Iron Maple for maple-sim
// https://github.com/Shenzhen-Robotics-Alliance/maple-sim
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.DriveTrainConstants.DRIVE_GEAR_RATIO;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and CANcoder.
 * Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFXReal extends ModuleIOTalonFX {
    // Queue to read inputs from odometry thread
    private final Queue<Angle> drivePositionQueue;
    private final Queue<Angle> turnPositionQueue;

    public ModuleIOTalonFXReal(SwerveModuleConstants constants) {
        super(constants);

        this.drivePositionQueue = OdometryThread.registerSignalSignal(super.drivePosition);
        this.turnPositionQueue = OdometryThread.registerSignalSignal(super.steerAbsolutePosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        // Update odometry inputs
        inputs.odometryDriveWheelRevolutions = drivePositionQueue.stream()
                .mapToDouble(angle -> angle.in(Rotations) / DRIVE_GEAR_RATIO)
                .toArray();
        inputs.odometrySteerPositions =
                turnPositionQueue.stream().map(Rotation2d::new).toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }
}

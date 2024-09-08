// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328

package frc.robot.subsystems.drive.IO;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.Queue;

/**
 * IO implementation for Pigeon2
 */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;
    private final StatusSignal<Double> yaw;
    private final Queue<Double> yawPositionInput;
    private final StatusSignal<Double> yawVelocity;

    public GyroIOPigeon2(SwerveDrivetrainConstants drivetrainConstants) {
        this(
                drivetrainConstants.Pigeon2Id,
                drivetrainConstants.CANbusName,
                drivetrainConstants.Pigeon2Configs
        );
    }

    public GyroIOPigeon2(int Pigeon2Id, String CANbusName, Pigeon2Configuration Pigeon2Configs) {
        pigeon = new Pigeon2(Pigeon2Id, CANbusName);
        if (Pigeon2Configs != null)
            pigeon.getConfigurator().apply(Pigeon2Configs);
        else
            pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);

        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        yawVelocity.setUpdateFrequency(100.0);
        yawPositionInput = OdometryThread.registerSignalInput(pigeon.getYaw());

        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawPositions = yawPositionInput.stream()
                .map(Rotation2d::fromDegrees)
                .toArray(Rotation2d[]::new);
        yawPositionInput.clear();
    }
}

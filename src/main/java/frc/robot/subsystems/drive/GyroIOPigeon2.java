// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

import java.util.Arrays;

/**
 * IO implementation for Pigeon2
 */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(0, Constants.ChassisConfigs.DEFAULT_CHASSIS_CANIVORE);
    private final StatusSignal<Double> yaw = pigeon.getYaw();
    private final OdometryThread.OdometryInput yawPositionInput;
    private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

    public GyroIOPigeon2() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(Module.ODOMETRY_FREQUENCY);
        yawVelocity.setUpdateFrequency(100.0);
        pigeon.optimizeBusUtilization();
        yawPositionInput = OdometryThread.registerSignalInput(pigeon.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawPositions = Arrays.stream(yawPositionInput.getValuesSincePreviousPeriod()).map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    }
}

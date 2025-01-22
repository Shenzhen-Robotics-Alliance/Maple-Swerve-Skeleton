package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.CustomPIDs.MaplePIDController;

public class DriveControlLoops {
    public static final LinearVelocity SWERVE_VELOCITY_DEADBAND = MetersPerSecond.of(0.03);
    public static final MaplePIDController.MaplePIDConfig CHASSIS_ROTATION_CLOSE_LOOP =
            new MaplePIDController.MaplePIDConfig(
                    Math.toRadians(300), Math.toRadians(90), 0, Math.toRadians(2), 0.05, true, 0);

    public static final MaplePIDController.MaplePIDConfig CHASSIS_TRANSLATION_CLOSE_LOOP =
            new MaplePIDController.MaplePIDConfig(3, 0.6, 0, 0.03, 0.05, false, 0);

    public static final double ROTATIONAL_LOOKAHEAD_TIME = 0.02, TRANSLATIONAL_LOOKAHEAD_TIME = 0.02;

    public static final boolean USE_TORQUE_FEEDFORWARD = true;
}

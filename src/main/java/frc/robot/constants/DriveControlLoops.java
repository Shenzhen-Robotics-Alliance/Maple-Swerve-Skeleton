package frc.robot.constants;

import frc.robot.utils.CustomPIDs.MaplePIDController;

public class DriveControlLoops {
    public static final MaplePIDController.MaplePIDConfig CHASSIS_ROTATION_CLOSE_LOOP =
            new MaplePIDController.MaplePIDConfig(
                    Math.toRadians(300), Math.toRadians(60), 0.03, Math.toRadians(3), 0.1, true, 0);

    public static final MaplePIDController.MaplePIDConfig CHASSIS_TRANSLATION_CLOSE_LOOP =
            new MaplePIDController.MaplePIDConfig(3, 0.6, 0, 0.03, 0.1, false, 0);

    public static final double ROTATIONAL_LOOKAHEAD_TIME = 0.05, TRANSLATIONAL_LOOKAHEAD_TIME = 0.02;

    public static final boolean USE_TORQUE_FEEDFORWARD = true;
}

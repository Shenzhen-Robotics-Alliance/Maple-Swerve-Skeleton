package frc.robot.constants;

import frc.robot.utils.CustomPIDs.MaplePIDController;

public class DriveControlLoops {
    public static final MaplePIDController.MaplePIDConfig CHASSIS_ROTATION_CLOSE_LOOP =
            new MaplePIDController.MaplePIDConfig(
                    Math.toRadians(400), Math.toRadians(90), 0.03, Math.toRadians(3), 0.04, true, 0);

    public static final MaplePIDController.MaplePIDConfig CHASSIS_TRANSLATION_CLOSE_LOOP =
            new MaplePIDController.MaplePIDConfig(2, 1.2, 0, 0.03, 0, false, 0);

    public static final double ROTATIONAL_LOOKAHEAD_TIME = 0.05, TRANSLATIONAL_LOOKAHEAD_TIME = 0.02;

    public static final boolean USE_TORQUE_FEEDFORWARD = true;
}

package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.CustomPIDs.MaplePIDController;

public class PitchConstants {
    public static final double GEAR_RATIO = 166.66;
    public static final double PITCH_LOWEST_ROTATION_RAD = Math.toRadians(24);
    public static final double PITCH_HIGHER_LIMIT_RAD = Math.toRadians(100);

    public static final double PITCH_KS = 0.03;
    public static final double PITCH_KG = 0.1;
    public static final double PITCH_KV = 3.2;
    public static final double PITCH_KA = 0.01;

    public static final MaplePIDController.MaplePIDConfig PITCH_PID = new MaplePIDController.MaplePIDConfig(
            7.5,
            Math.toRadians(26),
            0,
            Math.toRadians(2),
            0.05,
            false,
            0
    );

    public static final TrapezoidProfile.Constraints PITCH_PROFILE_CONSTRAIN = new TrapezoidProfile.Constraints(
            Math.toRadians(360), Math.toRadians(720)
    );
}

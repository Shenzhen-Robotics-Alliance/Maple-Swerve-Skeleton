package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.CustomPIDs.MaplePIDController;

public class ShooterConstants {
    public static final double[] ks = new double[] {0.05, 0.05};
    public static final double[] kv = new double[] {0.11613, 0.1145};
    public static final double kv_sim = 0.17;
    public static final double[] ka = new double[] {0.026625, 0.028551};

    public static final TrapezoidProfile.Constraints SPEED_RPM_CONSTRAINS = new TrapezoidProfile.Constraints(
            6000/0.5, 6000/0.3
    );

    public static final double TOLERANCE_RPM = 65;
    public static final MaplePIDController.MaplePIDConfig FLYWHEEL_PID_CONFIG_REV_PER_SEC = new MaplePIDController.MaplePIDConfig(
            7,
            30,
            0,
            2,
            0, false, 0
    );
}

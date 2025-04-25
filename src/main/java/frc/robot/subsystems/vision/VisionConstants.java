package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class VisionConstants {
    public static final Distance PRIMARY_ESTIMATOR_ODOMETRY_TRANSLATIONAL_STANDARD_ERROR = Centimeters.of(5);
    public static final Angle PRIMARY_ESTIMATOR_GYRO_ROTATIONAL_STANDARD_ERROR = Degrees.of(2);

    public static final Distance VISION_SENSITIVE_ESTIMATOR_ODOMETRY_TRANSLATIONAL_STANDARD_ERROR = Centimeters.of(30);
    public static final Angle VISION_SENSITIVE_ESTIMATOR_GYRO_ROTATIONAL_STANDARD_ERROR = Degrees.of(20);

    public static final Time POSE_BUFFER_DURATION = Seconds.of(2);
}

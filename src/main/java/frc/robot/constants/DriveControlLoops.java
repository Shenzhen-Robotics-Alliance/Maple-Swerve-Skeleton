package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import frc.robot.commands.drive.AutoAlignment;
import frc.robot.utils.CustomPIDs.MaplePIDController;

public class DriveControlLoops {
    public static final boolean ENABLE_SOFTWARE_CONSTRAIN = true;
    public static final AngularVelocity ANGULAR_VELOCITY_SOFT_CONSTRAIN = RotationsPerSecond.of(1.0);
    public static final AngularAcceleration ANGULAR_ACCELERATION_SOFT_CONSTRAIN = RotationsPerSecondPerSecond.of(2.4);
    public static final LinearVelocity MOVEMENT_VELOCITY_SOFT_CONSTRAIN = MetersPerSecond.of(5.2);
    public static final LinearVelocity AUTO_ALIGNMENT_VELOCITY_LIMIT = MetersPerSecond.of(2.5);
    public static final LinearAcceleration ACCELERATION_SOFT_CONSTRAIN = MetersPerSecondPerSecond.of(8.0);
    public static final LinearAcceleration AUTO_ALIGNMENT_ACCELERATION_LIMIT = MetersPerSecondPerSecond.of(5.0);
    public static final LinearVelocity MOVEMENT_VELOCITY_SOFT_CONSTRAIN_LOW = MetersPerSecond.of(2.0);
    public static final LinearAcceleration ACCELERATION_SOFT_CONSTRAIN_LOW = MetersPerSecondPerSecond.of(3.5);
    public static final AngularVelocity ANGULAR_VELOCITY_SOFT_CONSTRAIN_LOW = RotationsPerSecond.of(0.75);
    public static final AngularAcceleration ANGULAR_ACCELERATION_SOFT_CONSTRAIN_LOW =
            RotationsPerSecondPerSecond.of(1.5);

    public static final Time DISCRETIZE_TIME = Seconds.of(0.04);
    public static final LinearVelocity SWERVE_VELOCITY_DEADBAND = MetersPerSecond.of(0.03);
    public static final MaplePIDController.MaplePIDConfig CHASSIS_ROTATION_CLOSE_LOOP =
            new MaplePIDController.MaplePIDConfig(
                    Math.toRadians(360), Math.toRadians(90), 0, Math.toRadians(2), 0, true, 0);

    public static final MaplePIDController.MaplePIDConfig CHASSIS_TRANSLATION_CLOSE_LOOP =
            new MaplePIDController.MaplePIDConfig(3, 0.5, 0, 0.03, 0, false, 0);

    public static final Time ROTATIONAL_LOOKAHEAD_TIME_VISION = Milliseconds.of(40);
    public static final Time TRANSLATIONAL_LOOKAHEAD_TIME_VISION = Milliseconds.of(70);
    public static final Time ROTATIONAL_LOOKAHEAD_TIME_SENSOR_LESS = Milliseconds.of(20);
    public static final Time TRANSLATIONAL_LOOKAHEAD_TIME_SENSOR_LESS = Milliseconds.of(20);

    public static final boolean USE_TORQUE_FEEDFORWARD = true;

    public static final double AUTO_ALIGNMENT_TRANSITION_COMPENSATION_FACTOR = 0.2;
    public static final Distance AUTO_ALIGNMENT_SUCCESS_BIAS_TOLERANCE = Centimeters.of(3);
    public static final Distance AUTO_ALIGNMENT_SUCCESS_DISTANCE_TOLERANCE = Centimeters.of(6);
    public static final Angle AUTO_ALIGNMENT_SUCCESS_TOLERANCE_ROTATIONAL = Degrees.of(2);

    public static final Distance ROUGH_APPROACHT_POSE_TO_TARGET_DISTANCE = Meters.of(1.4);
    public static final Distance ROUGH_APPROACH_POSE_TO_TARGET_MARGIN = Centimeters.of(15);

    public static final AutoAlignment.AutoAlignmentConfigurations REEF_ALIGNMENT_CONFIG_AUTONOMOUS =
            new AutoAlignment.AutoAlignmentConfigurations(
                    Meters.of(0.1),
                    MetersPerSecond.of(1.4),
                    Meters.of(0.24),
                    MetersPerSecond.of(0),
                    MetersPerSecondPerSecond.of(2.0));

    public static final AutoAlignment.AutoAlignmentConfigurations REEF_ALIGNMENT_CONFIG =
            new AutoAlignment.AutoAlignmentConfigurations(
                    Meters.of(0.4),
                    MetersPerSecond.of(1.4),
                    Meters.of(0.8),
                    MetersPerSecond.of(0),
                    MetersPerSecondPerSecond.of(2.0));

    public static final AutoAlignment.AutoAlignmentConfigurations REEF_ALIGNMENT_CONFIG_FAST =
            new AutoAlignment.AutoAlignmentConfigurations(
                    Meters.of(0.4),
                    MetersPerSecond.of(2.0),
                    Meters.of(0.6),
                    MetersPerSecond.of(0.05),
                    MetersPerSecondPerSecond.of(3.2));

    public static final AutoAlignment.AutoAlignmentConfigurations ALGAE_ALIGNMENT_CONFIG =
            new AutoAlignment.AutoAlignmentConfigurations(
                    Meters.of(0.5),
                    MetersPerSecond.of(0.8),
                    Meters.of(0.8),
                    MetersPerSecond.of(0.05),
                    MetersPerSecondPerSecond.of(2.0));

    public static final AutoAlignment.AutoAlignmentConfigurations STATION_ALIGNMENT_CONFIG =
            new AutoAlignment.AutoAlignmentConfigurations(
                    Meters.of(0.6),
                    MetersPerSecond.of(2),
                    Meters.of(0.3),
                    MetersPerSecond.of(0.6),
                    MetersPerSecondPerSecond.of(4));
}

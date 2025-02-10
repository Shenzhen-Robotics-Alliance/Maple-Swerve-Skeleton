package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import frc.robot.commands.drive.AutoAlignment;
import frc.robot.utils.CustomPIDs.MaplePIDController;

public class DriveControlLoops {
    public static final boolean ENABLE_SOFTWARE_CONSTRAIN = true;
    public static final AngularVelocity ANGULAR_VELOCITY_SOFT_CONSTRAIN = RotationsPerSecond.of(0.75);
    public static final AngularAcceleration ANGULAR_ACCELERATION_SOFT_CONSTRAIN = RotationsPerSecondPerSecond.of(1);
    public static final LinearVelocity MOVEMENT_VELOCITY_SOFT_CONSTRAIN = DriveTrainConstants.CHASSIS_MAX_VELOCITY;
    public static final LinearAcceleration ACCELERATION_SOFT_CONSTRAIN = MetersPerSecondPerSecond.of(6);
    public static final LinearVelocity MOVEMENT_VELOCITY_SOFT_CONSTRAIN_LOW = MetersPerSecond.of(2);
    public static final LinearAcceleration ACCELERATION_SOFT_CONSTRAIN_LOW = MetersPerSecondPerSecond.of(3);

    public static final Time DISCRETIZE_TIME = Seconds.of(0.04);
    public static final LinearVelocity SWERVE_VELOCITY_DEADBAND = MetersPerSecond.of(0.03);
    public static final MaplePIDController.MaplePIDConfig CHASSIS_ROTATION_CLOSE_LOOP =
            new MaplePIDController.MaplePIDConfig(
                    Math.toRadians(300), Math.toRadians(90), 0, Math.toRadians(2), 0, true, 0);

    public static final MaplePIDController.MaplePIDConfig CHASSIS_TRANSLATION_CLOSE_LOOP =
            new MaplePIDController.MaplePIDConfig(3, 0.4, 0, 0.03, 0, false, 0);

    public static final double ROTATIONAL_LOOKAHEAD_TIME = 0.05, TRANSLATIONAL_LOOKAHEAD_TIME = 0.05;

    public static final boolean USE_TORQUE_FEEDFORWARD = true;

    public static final AutoAlignment.AutoAlignmentConfigurations REEF_ALIGNMENT_CONFIG_AUTONOMOUS =
            new AutoAlignment.AutoAlignmentConfigurations(
                    Meters.of(1),
                    MetersPerSecond.of(2),
                    MetersPerSecond.of(1),
                    Meters.of(0.6),
                    MetersPerSecond.of(0.15),
                    MetersPerSecondPerSecond.of(2.5));

    public static final AutoAlignment.AutoAlignmentConfigurations REEF_ALIGNMENT_CONFIG =
            new AutoAlignment.AutoAlignmentConfigurations(
                    Meters.of(1.1),
                    MetersPerSecond.of(2),
                    MetersPerSecond.of(1),
                    Meters.of(0.6),
                    MetersPerSecond.of(0.15),
                    MetersPerSecondPerSecond.of(2.5));

    public static final AutoAlignment.AutoAlignmentConfigurations STATION_ALIGNMENT_CONFIG =
            new AutoAlignment.AutoAlignmentConfigurations(
                    Meters.of(0.6),
                    MetersPerSecond.of(5),
                    MetersPerSecond.of(2),
                    Meters.of(0.3),
                    MetersPerSecond.of(0.6),
                    MetersPerSecondPerSecond.of(4));
}

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;
import frc.robot.generated.TunerConstants;

public class DriveTrainConfigs {
    /* dead configs, don't change them */
    public static final int ODOMETRY_CACHE_CAPACITY = 10;
    public static final double ODOMETRY_FREQUENCY = 300;
    public static final double ODOMETRY_WAIT_TIMEOUT_SECONDS = 0.05;
    public static final int SIMULATION_TICKS_IN_1_PERIOD = 6;

    public static final PathConstraints DRIVE_CONSTRAINS = new PathConstraints(
            TunerConstants.kSpeedAt12Volts,
            MetersPerSecondPerSecond.of(10.5),
            RotationsPerSecond.of(2.0),
            RotationsPerSecondPerSecond.of(3.5));

    public static final PathConstraints DRIVE_CONSTRAINS_LOW_SPEED = new PathConstraints(
            MetersPerSecond.of(2.0),
            MetersPerSecondPerSecond.of(5.0),
            RotationsPerSecond.of(2.0),
            RotationsPerSecondPerSecond.of(2.0));

    public static final 
}

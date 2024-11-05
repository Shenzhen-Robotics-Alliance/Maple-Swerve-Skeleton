package frc.robot.constants;

/** Configs for the driver's joystick See {@link frc.robot.utils.MapleJoystickDriveInput} */
public class JoystickConfigs {
    public static final double DEFAULT_TRANSLATIONAL_SENSITIVITY = 1;
    public static final double DEFAULT_ROTATIONAL_SENSITIVITY = 0.7;

    /**
     * the amount of time that the chassis waits after the pilot's last input, before it places all the swerve wheels to
     * standby-state (facing forward)
     */
    public static final double NON_USAGE_TIME_RESET_WHEELS = 1;

    /** */
    public static final double DEAD_BAND_WHEN_OTHER_AXIS_EMPTY = 0.02;

    public static final double DEAD_BAND_WHEN_OTHER_AXIS_FULL = 0.1;
    public static final double LINEAR_SPEED_INPUT_EXPONENT = 1.6;
    public static final double ROTATION_SPEED_INPUT_EXPONENT = 2;

    /**
     * the amount of time that the chassis needs to shift to the desired pilot motion it's sort of a "smooth out" of the
     * pilot's input this dramatically reduces over-current and brownouts
     */
    public static final double LINEAR_ACCELERATION_SMOOTH_OUT_SECONDS = 0.1;
    /** same thing for rotation */
    public static final double ANGULAR_ACCELERATION_SMOOTH_OUT_SECONDS = 0.1;

    /**
     * the amount of time that the chassis waits after the pilot's last rotational input, before it starts to "lock" its
     * rotation with PID
     */
    public static final double TIME_ACTIVATE_ROTATION_MAINTENANCE_AFTER_NO_ROTATIONAL_INPUT_SECONDS = 0.6;
}

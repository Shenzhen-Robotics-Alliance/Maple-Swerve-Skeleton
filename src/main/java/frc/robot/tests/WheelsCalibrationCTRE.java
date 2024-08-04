package frc.robot.tests;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.MapleMaths.Angles;

import java.io.IOException;

import static frc.robot.Constants.WheelCalibrationConfigs.WheelToBeCalibrated;
import static frc.robot.Constants.WheelCalibrationConfigs.wheelsToBeCalibrated;

public class WheelsCalibrationCTRE extends Command {

    private enum SteerWheelTurningDirection {
        NOT_INVERTED,
        INVERTED
    }
    private final SendableChooser<SteerWheelTurningDirection> wheelTurningDirectionSendableChooser = new SendableChooser<>();
    private final SendableChooser<WheelToBeCalibrated> wheelSendableChooser = new SendableChooser<>();
    private final String configName;
    private final MapleConfigFile calibrationFile;
    public WheelsCalibrationCTRE() {
        configName = "9976-2024-OffSeason";
        for (WheelToBeCalibrated wheelToBeCalibrated: Constants.WheelCalibrationConfigs.wheelsToBeCalibrated)
            wheelSendableChooser.addOption(wheelToBeCalibrated.name, wheelToBeCalibrated);
        wheelSendableChooser.setDefaultOption(Constants.WheelCalibrationConfigs.wheelsToBeCalibrated[0].name, Constants.WheelCalibrationConfigs.wheelsToBeCalibrated[0]);
        wheelSendableChooser.onChange(this::initHardware);

        SmartDashboard.putData("Calibration/Select Wheel to Calibrate", wheelSendableChooser);

        this.calibrationFile = new MapleConfigFile("ChassisWheelsCalibration", configName);

        wheelTurningDirectionSendableChooser.setDefaultOption(SteerWheelTurningDirection.NOT_INVERTED.name(), SteerWheelTurningDirection.NOT_INVERTED);
        wheelTurningDirectionSendableChooser.addOption(SteerWheelTurningDirection.INVERTED.name(), SteerWheelTurningDirection.INVERTED);
    }

    @Override
    public void initialize() {
        initHardware(wheelSendableChooser.getSelected());
        SmartDashboard.putData("Calibration/Steer Motor Turning Direction (Should be Spinning Counter-Clockwise)", wheelTurningDirectionSendableChooser);
        SmartDashboard.putData("Calibration/moveDrivingWheel", new InstantCommand(
                () -> drivingMotor.set(0.3)).finallyDo(() -> drivingMotor.set(0))
        );
        SmartDashboard.putData("Calibration/moveSteeringWheel", new InstantCommand(
                () -> steeringMotor.set(wheelSendableChooser.getSelected().steeringMotorInverted ? -0.2:0.2)).finallyDo(() -> steeringMotor.set(0))
        );

        SmartDashboard.putData("Calibration/save", new InstantCommand(this::writeConfigurationFile));
        finished = false;
    }


    private TalonFX drivingMotor, steeringMotor;
    private CANcoder canCoder;
    private boolean finished;

    private final XboxController controller = new XboxController(1);
    @Override
    public void execute() {
        if (finished) return;
        wheelSendableChooser.getSelected().steeringMotorInverted = switch (wheelTurningDirectionSendableChooser.getSelected()){
            case NOT_INVERTED -> false;
            case INVERTED -> true;
        };

        for (int i = 0; i < wheelsToBeCalibrated.length; i++) {
            final CANcoder canCoder = new CANcoder(wheelsToBeCalibrated[i].encoderID, Constants.SwerveDriveChassisConfigs.CHASSIS_CANBUS);
            SmartDashboard.putNumber("Calibration/" + wheelsToBeCalibrated[i].name + " encoder (rad): ", getCanCoderReadingRadian(canCoder));
        }

        SmartDashboard.putNumber("Calibration/Can Coder Reading (Rad)", getCanCoderReadingRadian(canCoder));
    }

    private double getCanCoderReadingRadian(CANcoder canCoder) {
        return Angles.simplifyAngle(canCoder.getAbsolutePosition().getValue() * Math.PI * 2);
    }

    private void writeConfigurationFile() {
        System.out.println("writing configs to usb");
        if (finished) return;
        final MapleConfigFile.ConfigBlock configBlock = calibrationFile.getBlock("GeneralInformation");
        configBlock.putIntConfig("gyroPort", Constants.ChassisDefaultConfigs.DEFAULT_GYRO_PORT);
        configBlock.putDoubleConfig("overallGearRatio", Constants.ChassisDefaultConfigs.DEFAULT_GEAR_RATIO);
        configBlock.putDoubleConfig("wheelRadiusMeters", Constants.ChassisDefaultConfigs.DEFAULT_WHEEL_RADIUS_METERS);
        configBlock.putDoubleConfig("horizontalWheelsMarginMeters", Constants.ChassisDefaultConfigs.DEFAULT_HORIZONTAL_WHEELS_MARGIN_METERS);
        configBlock.putDoubleConfig("verticalWheelsMarginMeters", Constants.ChassisDefaultConfigs.DEFAULT_VERTICAL_WHEELS_MARGIN_METERS);
        configBlock.putDoubleConfig("maxVelocityMetersPerSecond", Constants.ChassisDefaultConfigs.DEFAULT_MAX_VELOCITY_METERS_PER_SECOND);
        configBlock.putDoubleConfig("maxAccelerationMetersPerSecondSquared", Constants.ChassisDefaultConfigs.DEFAULT_MAX_ACCELERATION_METERS_PER_SQUARED_SECOND);
        configBlock.putDoubleConfig("maxAngularVelocityRadiansPerSecond", Math.toRadians(Constants.ChassisDefaultConfigs.DEFAULT_MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND));

        configBlock.putDoubleConfig("bumperWidthMeters", Constants.RobotPhysicsSimulationConfigs.DEFAULT_BUMPER_WIDTH_METERS);
        configBlock.putDoubleConfig("bumperLengthMeters", Constants.RobotPhysicsSimulationConfigs.DEFAULT_BUMPER_LENGTH_METERS);
        configBlock.putDoubleConfig("robotMassInSimulation", Constants.RobotPhysicsSimulationConfigs.DEFAULT_ROBOT_MASS);

        for (WheelToBeCalibrated wheelToBeCalibrated:Constants.WheelCalibrationConfigs.wheelsToBeCalibrated)
            saveConfigurationForCurrentWheel(wheelToBeCalibrated);

        try {
            calibrationFile.saveConfigToUSB();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        finished = true;
    }

    private void saveConfigurationForCurrentWheel(WheelToBeCalibrated wheelToBeCalibrated) {
        final MapleConfigFile.ConfigBlock configBlock = calibrationFile.getBlock(wheelToBeCalibrated.name);
        configBlock.putIntConfig("drivingMotorID", wheelToBeCalibrated.drivingMotorID);
        configBlock.putIntConfig("drivingMotorPortOnPDP", wheelToBeCalibrated.drivingMotorPortOnPDP);
        configBlock.putIntConfig("steeringMotorID", wheelToBeCalibrated.steeringMotorID);
        configBlock.putIntConfig("steeringMotorPortOnPDP", wheelToBeCalibrated.steeringMotorPortOnPDP);
        configBlock.putIntConfig("steeringEncoderID", wheelToBeCalibrated.encoderID);

        configBlock.putIntConfig("steeringMotorInverted", wheelToBeCalibrated.steeringMotorInverted ? 1 : 0);

        final CANcoder canCoder = new CANcoder(wheelToBeCalibrated.encoderID, Constants.SwerveDriveChassisConfigs.CHASSIS_CANBUS);
        configBlock.putDoubleConfig("steeringEncoderReadingAtOrigin", getCanCoderReadingRadian(canCoder));
    }

    private void initHardware(WheelToBeCalibrated wheelToBeCalibrated) {
        drivingMotor = new TalonFX(wheelToBeCalibrated.drivingMotorID, Constants.SwerveDriveChassisConfigs.CHASSIS_CANBUS);
        drivingMotor.setNeutralMode(NeutralModeValue.Coast);
        steeringMotor = new TalonFX(wheelToBeCalibrated.steeringMotorID, Constants.SwerveDriveChassisConfigs.CHASSIS_CANBUS);
        steeringMotor.setNeutralMode(NeutralModeValue.Coast);
        steeringMotor.setInverted(false);
        canCoder = new CANcoder(wheelToBeCalibrated.encoderID, Constants.SwerveDriveChassisConfigs.CHASSIS_CANBUS);
    }

    @Override public boolean isFinished() {return finished; }
}

package frc.robot.utils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;

public class MapleJoystickDriveInput {
    private final DoubleSupplier joystickXSupplier, joystickYSupplier, joystickOmegaSupplier;

    /**
     * @param joystickXSupplier the supplier of the x-axis of the joystick, positive is RIGHTWARDS
     * @param joystickYSupplier the supplier of the x-axis of the joystick, positive is DOWNWARDS
     * @param joystickOmegaSupplier the supplier of the omega-axis of the joystick, positive is RIGHTWARDS
     * */
    public MapleJoystickDriveInput(DoubleSupplier joystickXSupplier, DoubleSupplier joystickYSupplier, DoubleSupplier joystickOmegaSupplier) {
        this.joystickXSupplier = joystickXSupplier;
        this.joystickYSupplier = joystickYSupplier;
        this.joystickOmegaSupplier = joystickOmegaSupplier;
    }

    /**
     * reads the joystick inputs and calculate the chassis speeds
     * @return the chassis speeds requested by the pilot, driver-station-centric
     * */
    public ChassisSpeeds getJoystickChassisSpeeds(double chassisMaxVelocityMetersPerSec, double maxAngularVelocityRadPerSec) {
        return new ChassisSpeeds(); // TODO: write this method
    }

    /**
     *
     * */
    public double applySmartDeadBand(double axisValue, double otherAxisValue) {
        return 0;
    }

    public static MapleJoystickDriveInput leftHandedJoystick(CommandXboxController driverController) {
        return new MapleJoystickDriveInput(
                driverController::getLeftX,
                driverController::getLeftY,
                driverController::getRightX
        );
    }

    public static MapleJoystickDriveInput leftHandedJoystick(XboxController driverController) {
        return new MapleJoystickDriveInput(
                driverController::getLeftX,
                driverController::getLeftY,
                driverController::getRightX
        );
    }

    public static MapleJoystickDriveInput rightHandedJoystick(CommandXboxController driverController) {
        return new MapleJoystickDriveInput(
                driverController::getLeftX,
                driverController::getLeftY,
                driverController::getRightX
        );
    }

    public static MapleJoystickDriveInput rightHandedJoystick(XboxController driverController) {
        return new MapleJoystickDriveInput(
                driverController::getLeftX,
                driverController::getLeftY,
                driverController::getRightX
        );
    }
}

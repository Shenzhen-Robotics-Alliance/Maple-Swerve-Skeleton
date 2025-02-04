package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.MapleJoystickDriveInput;
import java.util.function.DoubleSupplier;

public interface DriverMap extends Subsystem {
    Trigger povUp();

    Trigger povDown();

    Trigger povLeft();

    Trigger povRight();

    Trigger resetOdometryButton();

    Trigger lockChassisWithXFormatButton();

    Trigger autoAlignmentButtonLeft();

    Trigger autoAlignmentButtonRight();

    Trigger faceToTargetButton();

    DoubleSupplier translationalAxisX();

    DoubleSupplier translationalAxisY();

    DoubleSupplier rotationalAxisX();

    DoubleSupplier rotationalAxisY();

    CommandGenericHID getController();

    default MapleJoystickDriveInput getDriveInput() {
        return new MapleJoystickDriveInput(
                this.translationalAxisX(), this.translationalAxisY(), this.rotationalAxisX());
    }

    default Command rumble(double seconds) {
        return runEnd(() -> getController().setRumble(GenericHID.RumbleType.kBothRumble, 1), () -> getController()
                        .setRumble(GenericHID.RumbleType.kBothRumble, 0))
                .withTimeout(seconds);
    }

    default Command rumbleLeftRight(double seconds) {
        return Commands.sequence(
                runOnce(() -> getController().setRumble(GenericHID.RumbleType.kLeftRumble, 1)),
                Commands.waitSeconds(seconds),
                runOnce(() -> {
                    getController().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                    getController().setRumble(GenericHID.RumbleType.kRightRumble, 1);
                }),
                Commands.waitSeconds(seconds),
                runOnce(() -> getController().setRumble(GenericHID.RumbleType.kBothRumble, 0)));
    }

    abstract class DriverXbox implements DriverMap {
        protected final CommandXboxController xboxController;

        protected DriverXbox(int port) {
            this.xboxController = new CommandXboxController(port);
        }

        @Override
        public Trigger povUp() {
            return xboxController.povUp();
        }

        @Override
        public Trigger povDown() {
            return xboxController.povDown();
        }

        @Override
        public Trigger povLeft() {
            return xboxController.povLeft();
        }

        @Override
        public Trigger povRight() {
            return xboxController.povRight();
        }

        @Override
        public Trigger resetOdometryButton() {
            return xboxController.start();
        }

        @Override
        public Trigger lockChassisWithXFormatButton() {
            return xboxController.x();
        }

        @Override
        public Trigger autoAlignmentButtonLeft() {
            return xboxController.leftBumper();
        }

        @Override
        public Trigger autoAlignmentButtonRight() {
            return xboxController.rightBumper();
        }

        @Override
        public Trigger faceToTargetButton() {
            return xboxController.leftBumper();
        }

        @Override
        public CommandGenericHID getController() {
            return xboxController;
        }
    }

    final class LeftHandedXbox extends DriverXbox {
        public LeftHandedXbox(int port) {
            super(port);
        }

        @Override
        public DoubleSupplier translationalAxisX() {
            return xboxController::getLeftX;
        }

        @Override
        public DoubleSupplier translationalAxisY() {
            return xboxController::getLeftY;
        }

        @Override
        public DoubleSupplier rotationalAxisX() {
            return xboxController::getRightX;
        }

        @Override
        public DoubleSupplier rotationalAxisY() {
            return xboxController::getRightY;
        }
    }

    class RightHandedXbox extends DriverXbox {
        public RightHandedXbox(int port) {
            super(port);
        }

        @Override
        public DoubleSupplier translationalAxisX() {
            return xboxController::getRightX;
        }

        @Override
        public DoubleSupplier translationalAxisY() {
            return xboxController::getRightY;
        }

        @Override
        public DoubleSupplier rotationalAxisX() {
            return xboxController::getLeftX;
        }

        @Override
        public DoubleSupplier rotationalAxisY() {
            return xboxController::getLeftY;
        }
    }

    abstract class DriverPS5 implements DriverMap {
        protected final CommandPS5Controller ps5Controller;

        public DriverPS5(int port) {
            this.ps5Controller = new CommandPS5Controller(port);
        }

        @Override
        public Trigger povUp() {
            return ps5Controller.povUp();
        }

        @Override
        public Trigger povDown() {
            return ps5Controller.povDown();
        }

        @Override
        public Trigger povLeft() {
            return ps5Controller.povLeft();
        }

        @Override
        public Trigger povRight() {
            return ps5Controller.povRight();
        }

        @Override
        public Trigger resetOdometryButton() {
            return ps5Controller.options();
        }

        @Override
        public Trigger lockChassisWithXFormatButton() {
            return ps5Controller.square();
        }

        @Override
        public Trigger autoAlignmentButtonLeft() {
            return ps5Controller.L1();
        }

        @Override
        public Trigger autoAlignmentButtonRight() {
            return ps5Controller.R1();
        }

        @Override
        public Trigger faceToTargetButton() {
            return ps5Controller.L1();
        }

        @Override
        public CommandGenericHID getController() {
            return ps5Controller;
        }
    }

    final class LeftHandedPS5 extends DriverPS5 {

        public LeftHandedPS5(int port) {
            super(port);
        }

        @Override
        public DoubleSupplier translationalAxisX() {
            return ps5Controller::getLeftX;
        }

        @Override
        public DoubleSupplier translationalAxisY() {
            return ps5Controller::getLeftY;
        }

        @Override
        public DoubleSupplier rotationalAxisX() {
            return ps5Controller::getRightX;
        }

        @Override
        public DoubleSupplier rotationalAxisY() {
            return ps5Controller::getRightY;
        }
    }

    final class RightHandedPS5 extends DriverPS5 {

        public RightHandedPS5(int port) {
            super(port);
        }

        @Override
        public DoubleSupplier translationalAxisX() {
            return ps5Controller::getRightX;
        }

        @Override
        public DoubleSupplier translationalAxisY() {
            return ps5Controller::getRightY;
        }

        @Override
        public DoubleSupplier rotationalAxisX() {
            return ps5Controller::getLeftX;
        }

        @Override
        public DoubleSupplier rotationalAxisY() {
            return ps5Controller::getLeftY;
        }
    }
}

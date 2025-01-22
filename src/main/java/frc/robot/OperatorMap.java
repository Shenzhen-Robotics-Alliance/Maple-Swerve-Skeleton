package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.MapleJoystickDriveInput;
import java.util.function.DoubleSupplier;

public interface OperatorMap {
    Trigger resetOdometryButton();

    Trigger lockChassisWithXFormatButton();

    Trigger autoAlignmentButton();

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

    abstract class OperatorXbox implements OperatorMap {
        protected final CommandXboxController xboxController;

        protected OperatorXbox(int port) {
            this.xboxController = new CommandXboxController(port);
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
        public Trigger autoAlignmentButton() {
            return xboxController.rightTrigger(0.5);
        }

        @Override
        public Trigger faceToTargetButton() {
            return xboxController.rightBumper();
        }

        @Override
        public CommandGenericHID getController() {
            return xboxController;
        }
    }

    final class LeftHandedXbox extends OperatorXbox {
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

    class RightHandedXbox extends OperatorXbox {
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

    abstract class OperatorPS5 implements OperatorMap {
        protected final CommandPS5Controller ps5Controller;

        public OperatorPS5(int port) {
            this.ps5Controller = new CommandPS5Controller(port);
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
        public Trigger autoAlignmentButton() {
            return ps5Controller.R1();
        }

        @Override
        public Trigger faceToTargetButton() {
            return ps5Controller.cross();
        }

        @Override
        public CommandGenericHID getController() {
            return ps5Controller;
        }
    }

    final class LeftHandedPS5 extends OperatorPS5 {

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

    final class RightHandedPS5 extends OperatorPS5 {

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

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.RobotMode;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.utils.CustomPIDs.MaplePIDController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

import static frc.robot.constants.ShooterConstants.*;

public class FlyWheels extends MapleSubsystem {
    private final FlyWheelIO[] IOs;
    private final FlyWheelsInputsAutoLogged[] inputs;

    public final SysIdRoutine[] sysIdRoutines;

    /* velocity RPM of the shooter is the position of the trapezoid profile */
    private final TrapezoidProfile speedRPMProfile;
    private final SimpleMotorFeedforward[] feedForwardRevPerSec;
    private final PIDController feedBackRevPerSec;
    private TrapezoidProfile.State currentStateRPM;
    private double goalRPM;
    public FlyWheels(FlyWheelIO[] IOs) {
        super("FlyWheels");
        this.IOs = IOs;

        this.feedBackRevPerSec =  new MaplePIDController(FLYWHEEL_PID_CONFIG_REV_PER_SEC);
        this.feedForwardRevPerSec = new SimpleMotorFeedforward[IOs.length];
        for (int i = 0; i < IOs.length; i++) {
            final double kvForCurrentWheel = Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
                    ? kv_sim : kv[i];
            this.feedForwardRevPerSec[i] = new SimpleMotorFeedforward(ks[i], kvForCurrentWheel, ka[i]);
        }

        this.inputs = new FlyWheelsInputsAutoLogged[IOs.length];
        this.sysIdRoutines = new SysIdRoutine[IOs.length];
        for (int i = 0; i < inputs.length; i++) {
            this.inputs[i] = new FlyWheelsInputsAutoLogged();
            this.sysIdRoutines[i] = new SysIdRoutine(
                    new SysIdRoutine.Config(null,null,null, this::logState),
                    new SysIdRoutine.Mechanism(voltageMeasureRunner(i), null, this)
            );
        }
        this.speedRPMProfile = new TrapezoidProfile(SPEED_RPM_CONSTRAINS);
        this.currentStateRPM = new TrapezoidProfile.State(0, 0);
        this.goalRPM = 0;

        setDefaultCommand(Commands.run(this::runIdle, this));
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        for (int i = 0; i < IOs.length; i++)
            flyWheelPeriodic(i);

        Logger.recordOutput("Shooter/FlyWheelsGoalRPM", goalRPM);
        SmartDashboard.putBoolean("FlyWheels Ready", flyWheelsReady());
        SmartDashboard.putNumber("FlyWheels Actual RPM", inputs[0].flyWheelVelocityRevsPerSec * 60);
    }

    private void flyWheelPeriodic(int index) {
        this.IOs[index].updateInputs(inputs[index]);
        Logger.processInputs("FlyWheels/" + index, inputs[index]);
        Logger.recordOutput("Shooter/FlyWheel"+index + " Measured RPM", inputs[index].flyWheelVelocityRevsPerSec * 60);
    }

    @Override
    public void onDisable() {
        for (FlyWheelIO io:IOs)
            io.runVoltage(0);
    }

    private void logState(SysIdRoutineLog.State state) {
        Logger.recordOutput("Shooter/FlyWheelsSysIdState", state.toString());
    }

    private Consumer<Measure<Voltage>> voltageMeasureRunner(int index) {
        return (voltageMeasure -> runVolts(index, voltageMeasure.in(Units.Volt)));
    }

    public void runIdle() {
        for (FlyWheelIO io : IOs) io.runVoltage(0);
        goalRPM = 0;
    }

    public void forceMaxRevert() {
        for (FlyWheelIO io : IOs) io.runVoltage(-12);
        goalRPM = 0;
    }

    private void runVolts(int index, double volts) {
        Logger.recordOutput("Shooter/FlyWheel" + index + "AppliedVolts", volts);
        IOs[index].runVoltage(volts);
    }

    public void runRPMProfiled(double rpm) {
        this.currentStateRPM = speedRPMProfile.calculate(
                Robot.defaultPeriodSecs,
                currentStateRPM,
                new TrapezoidProfile.State(rpm, 0)
        );
        this.goalRPM = rpm;
        runControlLoops();
    }

    public void runStaticRPMSetPoint(double rpm, double rateOfChangePerSec) {
        this.currentStateRPM = new TrapezoidProfile.State(rpm, rateOfChangePerSec);
        this.goalRPM = rpm;
        runControlLoops();
    }

    private void runControlLoops() {
        Logger.recordOutput("Shooter/Control Loop Setpoint (RPM)", currentStateRPM.position);

        for (int i = 0; i < IOs.length; i++) {
            final double flyWheelVelocityRevPerSec = currentStateRPM.position / 60,
                    flyWheelAccelerationRevPerSec = currentStateRPM.velocity / 60,
                    feedForwardVoltage = feedForwardRevPerSec[i].calculate(
                            flyWheelVelocityRevPerSec,
                            flyWheelAccelerationRevPerSec
                    );
            final double feedBackVoltage = feedBackRevPerSec.calculate(
                    inputs[i].flyWheelVelocityRevsPerSec,
                    flyWheelVelocityRevPerSec
            );
            runVolts(i, feedBackVoltage + feedForwardVoltage);
        }
    }

    @AutoLogOutput(key = "Shooter/FlyWheelsReady")
    public boolean flyWheelsReady() {
        for (FlyWheelIO.FlyWheelsInputs input:inputs)
            if (Math.abs(input.flyWheelVelocityRevsPerSec * 60 - goalRPM) > TOLERANCE_RPM)
                return false;
        return true;
    }
}

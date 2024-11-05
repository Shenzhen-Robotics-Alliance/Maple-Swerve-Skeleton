package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/** command */
public class CommandOnFly extends Command {
    private final Supplier<Command> supplier;

    protected CommandOnFly(Supplier<Command> supplier) {
        super();
        this.supplier = supplier;
    }

    private Command command = null;

    @Override
    public void initialize() {
        command = supplier.get();
        super.addRequirements(command.getRequirements().toArray(new Subsystem[0]));
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}

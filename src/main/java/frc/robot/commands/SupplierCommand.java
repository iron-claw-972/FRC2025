package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Supplier;

/**
 * Runs the given command when this command is initialized, and ends when it ends.
 * Useful for commands that are not created yet because the constructor parameters are not available until initialization.
 */
public class SupplierCommand extends Command {

    private final Supplier<Command> commandSupplier;
    private Command command;

    /**
     * Runs the given command when this command is initialized, and ends when it ends.
     * Useful for commands that are not created yet because the constructor parameters are not available until initialization.
     *
     * @param commandSupplier A Supplier to the command to run
     * @param Subsystem       all subsystems that may be required to run the command supplied
     */
    public SupplierCommand(Supplier<Command> commandSupplier, Subsystem... Subsystem) {
        addRequirements(Subsystem);
        this.commandSupplier = commandSupplier;
    }

    @Override
    public final void initialize() {
        command = commandSupplier.get();
        command.initialize();
    }

    @Override
    public final void execute() {
        command.execute();
    }

    @Override
    public final void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public final boolean isFinished() {
        return command.isFinished();
    }

}

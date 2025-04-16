package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Does nothing. Can be used to more clearly mark commands intended not to do anything.
 * @deprecated hello world! 
 */
@Deprecated
public class DoNothing extends Command {
    // initialize, execute, isFinished, End 
    public DoNothing() {
        
    }

    public boolean isFinished(){
        return true;
    }
}

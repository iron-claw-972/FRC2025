package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;

/**
 * Command to eject coral.
 * Wants coral to be present.
 */
public class OuttakeCoralBasic extends Command {
    private Outtake outtake;

    // states the outtake may take
    enum State { EMPTY, LOADED, MOVING, JAMMED, REVERSING, DONE }

    private State state;

    // counter to measure time in ticks (every 20 milliseconds);
    private int ticks;

    public OuttakeCoralBasic(Outtake outtake){
        this.outtake = outtake;
        addRequirements(outtake);
    }

    @Override
    public void initialize(){
        // set the timer to zero
        ticks = 0;

        if (outtake.coralLoaded()) {
            // coral is present, ejecting makes sense
            outtake.outtake();
            state = State.LOADED;
        }
        else {
            // no coral present, ejecting does not make sense
            state = State.DONE;
        }
    }

    public void execute(){
        // bump the timer
        ticks++;

        switch (state) {
            case EMPTY:
            // do not need to do anything
            break;

            case LOADED:
            // waiting for ejected to become true
            if (outtake.coralEjected()) {
                state = State.MOVING;
            }
            break;

            case MOVING:
            // waiting for ejected to become false (success)
            if(!outtake.coralEjected()){
                state = State.DONE;
            }
            // waiting for a timeout; 100 ticks is 2 seconds.
            if (ticks > 25) {
                state = State.JAMMED;
            }
            break;

            case JAMMED:
            // reverse the motor
            outtake.setMotor(-0.15);
            state = State.REVERSING;
            break;

            case REVERSING:
            // waiting for ejected to be false
            if(!outtake.coralEjected()){
                state = State.DONE;
                outtake.stop();
            }
            break;

            case DONE:
            break;
        }
    }

    public boolean isFinished(){
        return state == State.DONE; 
    }

    public void end(boolean interrupted){
        outtake.stop();
    }
}

package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;

/**
 * Command to eject coral.
 * Wants coral to be present.
 * This should generally be accompanied by a time out!!!
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

        if (outtake.coralLoaded()) {
            // coral is present, ejecting makes sense
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
        /*if(ticks>200){
            state = State.DONE;
        }
            */

        switch (state) {
            case EMPTY:
            // do not need to do anything
            break;

            case LOADED:
            // waiting for ejected to become true
            // FIXME: is this supposed to be coralLoaded????
            if (outtake.coralEjecting()) {
                ticks = 0;
                state = State.MOVING;
                outtake.outtake();
            }
            break;

            case MOVING:
            // waiting for ejected to become false (success)
            if(!outtake.coralEjecting()){
                state = State.DONE;
            }
            // waiting for a timeout; 100 ticks is 2 seconds.
            if (ticks > 25) {
                state = State.JAMMED;
            }
            break;

            case JAMMED:
            // reverse the motor, at -0.1: sometimes did not have the power to reverse, at -0.15: ejected all the way back, hit the funnel
            outtake.setMotor(-0.125);

            state = State.REVERSING;
            break;

            case REVERSING:
            // waiting for ejected to be false
            if(!outtake.coralEjected()){
                state = State.LOADED;
                // outtake.stop();
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

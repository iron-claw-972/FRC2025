package frc.robot.commands.gpm;


import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;


/**
 * Command to eject coral.
 * Wants coral to be present.
 */
public class OuttakeCoralBasic extends Command {
    public static final double L4_SPEED = 0.15;
    public static final double OUTTAKE_SPEED = 0.4;

    private Outtake outtake;

    // states the outtake may take
    private enum State {LOADED, MOVING, REVERSING, DONE }

    private State state;

    // counter to measure time in ticks (every 20 milliseconds);
    private int ticks;

    private BooleanSupplier l4Supplier;

    public OuttakeCoralBasic(Outtake outtake, BooleanSupplier l4Supplier){
        this.outtake = outtake;
        this.l4Supplier = l4Supplier;
        addRequirements(outtake);
    }


    @Override
    public void initialize(){
        ticks = 0;


        if (outtake.coralLoaded()) {
            // coral is present, ejecting makes sense
            state = State.LOADED;
            // wheels start spinning
            outtake.setMotor(l4Supplier.getAsBoolean() ? L4_SPEED : OUTTAKE_SPEED);
        }
        else {
            // no coral present, ejecting does not make sense
            state = State.DONE;
        }
    }


    public void execute(){
        // bump the timer
        ticks++;


        // if the outtake goes for too long, just give up. Something is seriously wrong.
        // 100 ticks is 2 seconds; autonomous is only 15 seconds
        if(ticks > 100){
            state = State.DONE;
        }


        switch (state) {
            case LOADED:
            // waiting for ejected to become true
            if (outtake.coralEjecting()) {
                // at this point, ticks represents how long it took to move the coral to the ejecting sensor.
                SmartDashboard.putNumber("Coral Ejection Time", ticks * 0.020);


                // reset the timer
                ticks = 0;
                // we know the coral is moving
                state = State.MOVING;
            }
            break;


            case MOVING:
            // waiting for ejected to become false (success)
            if (!outtake.coralEjecting()){
                // coral has gone all the way through.
                SmartDashboard.putNumber("Coral Transit Time", ticks * 0.020);
                state = State.DONE;
            }
            // waiting for a timeout; 13 ticks is 0.26 seconds. It only takes 0.18 seconds to eject a coral.
            if (ticks > 13) {
                outtake.reverse();


                SmartDashboard.putNumber("Coral Transit Time", ticks * 0.020);


                state = State.REVERSING;
            }
            break;


            case REVERSING:
            // waiting for ejected to be false
            if (!outtake.coralEjecting()){
                // we are done. When command finishes, the motor will be stopped.
                state = State.DONE;
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



package frc.robot.commands.gpm;


import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;


/**
 * Command to eject coral.
 * Wants coral to be present.
 */
public class OuttakeCoralBasic extends Command {
    public static final double L4_SPEED = 0.7;
    public static final double OUTTAKE_SPEED = 0.2;

    private Outtake outtake;

    // counter to measure time in ticks (every 20 milliseconds);
    private Timer timer = new Timer();

    private BooleanSupplier l4Supplier;

    public OuttakeCoralBasic(Outtake outtake, BooleanSupplier l4Supplier){
        this.outtake = outtake;
        this.l4Supplier = l4Supplier;
        addRequirements(outtake);
    }


    @Override
    public void initialize(){
        timer.restart();
        outtake.setMotor(l4Supplier.getAsBoolean() ? L4_SPEED : OUTTAKE_SPEED);
    }

    public boolean isFinished(){
        return timer.hasElapsed(0.5);
    }


    public void end(boolean interrupted){
        outtake.stop();
    }
}



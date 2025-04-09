package frc.robot.commands.gpm;


import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;


/**
 * Command to eject coral.
 * Wants coral to be present.
 */
public class OuttakeCoralBasic extends Command {
    public static final double L1_SPEED = -0.2;
    public static final double L4_SPEED = -0.8;
    public static final double OUTTAKE_SPEED = -0.45;

    private Outtake outtake;

    // counter to measure time in ticks (every 20 milliseconds);
    private Timer timer = new Timer();

    private BooleanSupplier l4Supplier;
    private BooleanSupplier l1Supplier;

    public OuttakeCoralBasic(Outtake outtake, BooleanSupplier l4Supplier, BooleanSupplier l1Supplier){
        this.outtake = outtake;
        this.l4Supplier = l4Supplier;
        this.l1Supplier = l1Supplier;
        addRequirements(outtake);
    }


    @Override
    public void initialize(){
        timer.restart();
        boolean l4 = l4Supplier.getAsBoolean();
        boolean l1 = !l4 && l1Supplier.getAsBoolean();
        outtake.setMotor(l4 ? L4_SPEED : l1 ? L1_SPEED : OUTTAKE_SPEED);
    }

    public boolean isFinished(){
        return timer.hasElapsed(0.25);
    }


    public void end(boolean interrupted){
        outtake.stop();
    }
}



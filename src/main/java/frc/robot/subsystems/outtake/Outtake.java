package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Abstract class for the outtake. All commands should use this subsystem
 */
public abstract class Outtake extends SubsystemBase {
    /** Coral detected before the rollers */
    protected DIOSim dioInputLoaded;
    /** Coral detected after the rollers */
    protected DIOSim dioInputEjecting;

    protected abstract double getMotorSpeed();

    public void simulationPeriodic(){}

    /** Set the motor power to move the coral */
    public abstract void setMotor(double power);

    /** stop the coral motor */
    public void stop() {
        setMotor(0);
    }

    /** start spinning the rollers to eject the coral */
    public abstract void outtake();

    public abstract boolean coralLoaded();

    /**
     *  Coral is at the ejecting beam break sensor.
     * @return coral is interrupting the beam breaker.
     */
    public abstract boolean coralEjecting();

    public abstract void reverse();

    public void removeAlgae(){
        setMotor(-0.6);
    }

    public void intakeAlgaeReef() {
        setMotor(-0.6);
    }

    public void outtakeAlgae() {
        setMotor(0.9);
    }
}

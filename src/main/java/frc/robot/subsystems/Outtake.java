package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Abstract class for the outtake. All commands should use this subsystem
 */
public abstract class Outtake extends SubsystemBase {
    private int ticks = 0;
    /** Coral detected before the rollers */
    protected DIOSim dioInputLoaded;
    /** Coral detected after the rollers */
    protected DIOSim dioInputEjecting;

    protected abstract double getMotorSpeed();

    public void simulationPeriodic() {
        ticks++;

        if (getMotorSpeed() > 0.05) {
            if (ticks > 250) {
                ticks = 0;
            }
            // motor is outtaking
            // motor is spinning, ejecting will be true. after 0.14 seconds
            if (ticks == 7) {
                dioInputEjecting.setValue(false);
            }
            if (ticks == 14) {
                // after 0.14 seconds
                dioInputLoaded.setValue(true);
            }
            if (ticks == 16) {
                // after 0.18 seconds
                dioInputEjecting.setValue(true);
            }
        }

        if (ticks == 250) {
            // make coral appear again (set to true)
            dioInputLoaded.setValue(false);
        }
    }

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
}

package frc.robot.subsystems;





import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Abstract class for the outtake. All commands should use this subsystem
 */
public class Outtake extends SubsystemBase {
    private int ticks = 0;
    private boolean coralPresent = true;
    private double m_power = 0;

    
    public void periodic(){
        // when coral is ejecting, loading is true & ejecting is true. when coral shoots out, loading is false & ejecting is false
        ticks++;
        SmartDashboard.putBoolean("Coral loaded", coralLoaded());
        SmartDashboard.putBoolean("Coral ejected", coralEjecting());
        if (m_power > 0.05) {
            // 25 ticks is 500 miliseconds
            if (ticks == 25) {
                coralPresent = false;
            }
        }

 }

    /** Set the motor power to move the coral */
    public void setMotor(double power) {
        m_power = power;
    }

    /** stop the coral motor */
    public void stop(){
        setMotor(0);
    }


    /** start spinning the rollers to eject the coral */
    public void outtake() {
     setMotor(0.2);  
     // start the counter 
     ticks = 0;
    }


    public boolean coralLoaded() {
        return coralPresent;
    }


    /**
     *  Coral is at the ejecting beam break sensor.
     * @return coral is interrupting the beam breaker.
     */
    public boolean coralEjecting() {
        // 20 ticks is 400 milliseconds. 30 ticks is 600 milliseconds
        return ticks > 20 && ticks < 30;
    }


    public void reverse() {
        setMotor(-0.15);
    }

    public boolean isFinished() {
        // 40 ticks is 800 milliseconds
        return ticks > 40;
    }

    public void fakeLoad() {
        coralPresent = true;
    }
}

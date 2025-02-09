package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class Outtake extends SubsystemBase {

    private SparkFlex  motor = new SparkFlex(IdConstants.OUTTAKE_MOTOR, MotorType.kBrushless);
    private double power;

    /** Coral detected before the rollers */
    private DigitalInput digitalInputLoaded = new DigitalInput(9);
    private DIOSim dioInputLoaded;
    /** Coral detected after the rollers */
    private DigitalInput digitalInputEjecting = new DigitalInput(8);
    private DIOSim dioInputEjecting;
    private int ticks = 0;

    public Outtake(){
        motor.configure(new SparkFlexConfig()
            .inverted(true)
            .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        if (RobotBase.isSimulation()){
            // object that will control the loaded sensor 
            dioInputLoaded = new DIOSim(digitalInputLoaded);
            // object that will control the ejecting sensor
            dioInputEjecting = new DIOSim(digitalInputEjecting);
            // assume coral is loaded
            dioInputLoaded.setValue(false);
            // we are not ejecting
            dioInputEjecting.setValue(true);
        }
    }

    @Override
    public void periodic(){
        motor.set(power);
        SmartDashboard.putBoolean("Coral loaded", coralLoaded());
        SmartDashboard.putBoolean("Coral ejected", coralEjected());
    }

    @Override
    public void simulationPeriodic(){
        // when coral is ejecting, loading is true & ejecting is true. when coral shoots out, loading is false & ejecting is false
        ticks++;

        if (motor.get() > 0.05) {
            if (ticks > 250) {
                ticks = 0;
            }
            // motor is outtaking
            // motor is spinning, ejecting will be true. after 0.14 seconds
            if (ticks ==7) {
                dioInputEjecting.setValue(false);
            }
            if (ticks == 14){
                // after 0.14 seconds
                dioInputLoaded.setValue(true);
            }
            if (ticks == 16){
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
    public void setMotor(double power){
        this.power = power;
    }

    /** stop the coral motor */
    public void stop(){
        setMotor(0);
    }

    /** start spinning the rollers to eject the coral */
    public void outtake(){
        // assumes the coral is present
        // if the coral is not present, we should not bother to spin the rollers
        setMotor(SmartDashboard.getNumber("wheel speed", 0));
        // this starts the motor... what needs to be done later?
    }

    public boolean coralLoaded(){
       return !digitalInputLoaded.get();
    }

    /**
     *  Coral is at the ejecting beam break sensor.
     * @return coral is interrupting the beam breaker.
     */ 
    public boolean coralEjecting() {
        return !digitalInputEjecting.get();
    }

    public void reverse(){
        setMotor(-0.2);
    }

    public boolean isSimulation(){
        return RobotBase.isSimulation();
    }
}

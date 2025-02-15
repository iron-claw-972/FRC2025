package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.IdConstants;


public class OuttakeComp extends Outtake {

    private TalonFX  motor = new TalonFX(IdConstants.OUTTAKE_MOTOR_COMP );
    private double power;


    /** Coral detected before the rollers */
    private DigitalInput digitalInputLoaded = new DigitalInput(IdConstants.OUTTAKE_DIO_LOADED);
    private DIOSim dioInputLoaded;
    /** Coral detected after the rollers */
    private DigitalInput digitalInputEjecting = new DigitalInput(IdConstants.OUTTAKE_DIO_EJECTING);
    private DIOSim dioInputEjecting;

    public OuttakeComp(){
        // TODO: configure Kraken

        // build simulation
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
        SmartDashboard.putNumber("wheel speed",0);
    }

    @Override
    protected double getMotorSpeed() {
        return power;
    }

    @Override
    public void periodic(){
        motor.set(power);
        SmartDashboard.putBoolean("Coral loaded", coralLoaded());
        SmartDashboard.putBoolean("Coral ejected", coralEjecting());
    }

    /** Set the motor power to move the coral */
    public void setMotor(double power){
        this.power = power;
    }

    /** start spinning the rollers to eject the coral */
    public void outtake(){
        // assumes the coral is present
        // if the coral is not present, we should not bother to spin the rollers
        setMotor(SmartDashboard.getNumber("wheel speed", 0.2));
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

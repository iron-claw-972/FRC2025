package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.constants.IdConstants;


public class OuttakeComp extends Outtake {

    private TalonFX  motor = new TalonFX(IdConstants.OUTTAKE_MOTOR_COMP);
    private double power;


    /** Coral detected before the rollers */
    private DigitalInput digitalInputLoaded = new DigitalInput(IdConstants.OUTTAKE_DIO_LOADED);
    /** Coral detected after the rollers */
    private DigitalInput digitalInputEjecting = new DigitalInput(IdConstants.OUTTAKE_DIO_EJECTING);

    public OuttakeComp(){
        motor.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        );

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
        //SmartDashboard.putNumber("wheel speed",0);
    }

    @Override
    protected double getMotorSpeed() {
        return power;
    }

    @Override
    public void periodic(){
        motor.set(power);
        //  SmartDashboard.putBoolean("Coral loaded", coralLoaded());
        //  SmartDashboard.putBoolean("Coral ejected", coralEjecting());
    }

    /** Set the motor power to move the coral */
    public void setMotor(double power){
        this.power = power;
    }

    /** start spinning the rollers to eject the coral */
    public void outtake(){
        // assumes the coral is present
        // if the coral is not present, we should not bother to spin the rollers
        setMotor(0.3);
        // this starts the motor... what needs to be done later?
    }


    public boolean coralLoaded(){
       return !digitalInputLoaded.get();//digitalInputEjecting.get();
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

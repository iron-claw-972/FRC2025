package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.IdConstants;

public class OuttakeComp extends Outtake {

    private int ticks = 0;
    private TalonFX motor = new TalonFX(IdConstants.OUTTAKE_MOTOR_COMP);

    /** Coral detected before the rollers */
    private DigitalInput digitalInputLoaded = new DigitalInput(IdConstants.OUTTAKE_DIO_LOADED);
    private DIOSim dioInputLoaded;
    /** Coral detected after the rollers */
    private DigitalInput digitalInputEjecting = new DigitalInput(IdConstants.OUTTAKE_DIO_EJECTING);
    private DIOSim dioInputEjecting;

    public OuttakeComp(){
        // TODO: configure Kraken
        var talonFXConfigurator = motor.getConfigurator();
        var motorConfigs = new MotorOutputConfigs();
        // set invert to CW+ and apply config change
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfigurator.apply(motorConfigs);   

        // build simulation
        if (RobotBase.isSimulation()){
            // object that will control the loaded sensor
            dioInputLoaded = new DIOSim(digitalInputLoaded);
            // object that will control the ejecting sensor
            dioInputEjecting = new DIOSim(digitalInputEjecting);
            // assume coral is loaded
            dioInputLoaded.setValue(false);
            System.out.println("setValue(true)");
            // we are not ejecting
            dioInputEjecting.setValue(true);
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Coral loaded", coralLoaded());
        SmartDashboard.putBoolean("Coral ejected", coralEjecting());

        if (motor.get() > 0.05) {
            ticks++;
        }
    }

    @Override
    public void simulationPeriodic() {
        // after 600 milliseconds, no longer loaded
        if (ticks == 30) {
            // coral is no longer seen by the loaded sensor
            dioInputLoaded.setValue(true);
        }
    }

    /** Set the motor power to move the coral */
    public void setMotor(double power){
        motor.set(power);
    }

    /** start spinning the rollers to eject the coral */
    public void outtake(){
        // assumes the coral is present
        // if the coral is not present, we should not bother to spin the rollers
        // setMotor(SmartDashboard.getNumber("wheel speed", 0.2));
        setMotor(0.2);
        // this starts the motor... what needs to be done later?
        ticks = 0;
        System.out.println("power " + motor.get());
    }

    @Override
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

    public boolean isFinished() {
        return ticks > 50;
    }

    public void fakeLoad() {
        if (RobotBase.isSimulation()) {
            // loads the coral
            dioInputLoaded.setValue(false);
        }
    }

    public void close() {
        motor.close();
        digitalInputLoaded.close();
        digitalInputEjecting.close();
    }
}

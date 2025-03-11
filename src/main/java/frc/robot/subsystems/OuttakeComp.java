package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.IdConstants;


public class OuttakeComp extends Outtake {

    private TalonFX  motor = new TalonFX(IdConstants.OUTTAKE_MOTOR_COMP);
    private double power;

    /** Coral detected after the rollers */
    //TODO: im not sure if we have a replacement sensor for this
    private DigitalInput digitalInputEjecting = new DigitalInput(IdConstants.OUTTAKE_DIO_EJECTING);

    /** Coral detected before the rollers */
    private final ColorSensorV3 colorSensor = new ColorSensorV3(IdConstants.i2cPort);

    public OuttakeComp(){
        motor.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        );

        // build simulation
        if (RobotBase.isSimulation()){
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
        SmartDashboard.putNumber("Proximity", getProximity());

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

    public int getProximity() {
        return colorSensor.getProximity();  // Returns 0 (far) to ~2047 (very close)
    }

    // coral detection from color sensor
    public boolean coralLoaded() {
        //this is about 1/2inch away -- might have to change based on placement
        if (getProximity() > 800) {
            return true;
        }
        return false;
    }
}

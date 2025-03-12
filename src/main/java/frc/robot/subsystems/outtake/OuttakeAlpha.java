package frc.robot.subsystems.outtake;


import com.revrobotics.ColorSensorV3;
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
import frc.robot.constants.IdConstants;


public class OuttakeAlpha extends Outtake {

    private SparkFlex  motor = new SparkFlex(IdConstants.OUTTAKE_MOTOR_ALPHA, MotorType.kBrushless);
    private double power;


    /** Coral detected before the rollers */
    private final ColorSensorV3 colorSensor = new ColorSensorV3(IdConstants.i2cPort);
    /** Coral detected after the rollers */
    private DigitalInput digitalInputEjecting = new DigitalInput(IdConstants.OUTTAKE_DIO_EJECTING);



    public OuttakeAlpha(){
        motor.configure(new SparkFlexConfig()
            .inverted(true)
            .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        if (RobotBase.isSimulation()){
            // object that will control the ejecting sensor
            dioInputEjecting = new DIOSim(digitalInputEjecting);
            // assume coral is loaded
            dioInputLoaded.setValue(false);
            // we are not ejecting
            dioInputEjecting.setValue(true);
        }
    }

    @Override
    protected double getMotorSpeed(){
        return motor.get();
    }

    @Override
    public void periodic(){
        motor.set(power);
        // SmartDashboard.putBoolean("Coral loaded", coralLoaded());
        // SmartDashboard.putBoolean("Coral ejected", coralEjecting());

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

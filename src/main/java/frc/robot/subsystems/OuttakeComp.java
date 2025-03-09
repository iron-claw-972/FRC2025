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

    /** Coral detected before the rollers */
    private DigitalInput digitalInputLoaded = new DigitalInput(IdConstants.OUTTAKE_DIO_LOADED);
    /** Coral detected after the rollers */
    private DigitalInput digitalInputEjecting = new DigitalInput(IdConstants.OUTTAKE_DIO_EJECTING);

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    public int getProximity() {
        return colorSensor.getProximity();  // Returns 0 (far) to ~2047 (very close)
    }

    private final ColorMatch colorMatcher = new ColorMatch();

    // Define target colors (adjust based on calibration)
    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

    public String detectColor() {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            return "Blue";
        } else if (match.color == kGreenTarget) {
            return "Green";
        } else if (match.color == kRedTarget) {
            return "Red";
        } else if (match.color == kYellowTarget) {
            return "Yellow";
        } else {
            return "Unknown";
        }
    }

    public OuttakeComp(){
        motor.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        );
        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kYellowTarget);

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
        SmartDashboard.putNumber("Proximity", getProximity());
        SmartDashboard.putString("Color", detectColor());

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

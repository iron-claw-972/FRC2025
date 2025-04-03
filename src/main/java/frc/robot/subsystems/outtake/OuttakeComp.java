package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.hal.I2CJNI;
import frc.robot.constants.IdConstants;

public class OuttakeComp extends Outtake {

    private TalonFX  motor = new TalonFX(IdConstants.OUTTAKE_MOTOR_COMP);
    private double power;

    /** Coral detected before the rollers */
    private ColorSensorV3 colorSensor = new ColorSensorV3(IdConstants.i2cPort);

    OuttakeIOIntakesAutoLogged inputs = new OuttakeIOIntakesAutoLogged();

    public OuttakeComp(){
        motor.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        );
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

        inputs.motorVelocity = motor.getVelocity().getValueAsDouble();
        Logger.processInputs("Outtake", inputs);
        //Logger.recordOutput("Outtake/Sensor", getProximity());
        //Logger.recordOutput("Outtake/SensorConnected", colorSensor.isConnected());
    }

    /** Set the motor power to move the coral */
    public void setMotor(double power){
        this.power = power;
    }

    /** start spinning the rollers to eject the coral */
    public void outtake(){
        setMotor(-0.3);
    }

    /**
     *  Coral is in the outtake.
     * @return The same thing as coralLoaded(), for compatibility with previous code
     */
    public boolean coralEjecting() {
        return coralLoaded();
    }


    public void reverse(){
        setMotor(0.2);
    }

    public int getProximity() {
        inputs.proximity = colorSensor.getProximity();
        if (inputs.proximity > 0){
            return inputs.proximity;
        }
        else{
            I2CJNI.i2CClose(1);
            colorSensor = new ColorSensorV3(IdConstants.i2cPort);
            return inputs.proximity = colorSensor.getProximity();  // Returns 0 (far) to ~2047 (very close)
        }
    }

    // coral detection from color sensor
    public boolean coralLoaded() {
        return getProximity() > 2000;
    }
}

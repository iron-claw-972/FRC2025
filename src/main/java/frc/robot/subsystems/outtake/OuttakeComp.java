package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;

import frc.robot.constants.IdConstants;

public class OuttakeComp extends Outtake {

    private TalonFX  motor = new TalonFX(IdConstants.OUTTAKE_MOTOR_COMP);
    private double power;

    /** Coral detected before the rollers */
    private final ColorSensorV3 colorSensor = new ColorSensorV3(IdConstants.i2cPort);

    OuttakeIOIntakesAutoLogged inputs = new OuttakeIOIntakesAutoLogged();

    public OuttakeComp(){
        motor.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
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
        inputs.proximity = getProximity();
        Logger.processInputs("Outtake", inputs);
    }

    /** Set the motor power to move the coral */
    public void setMotor(double power){
        this.power = power;
    }

    /** start spinning the rollers to eject the coral */
    public void outtake(){
        setMotor(0.3);
    }

    /**
     *  Coral is at the ejecting beam break sensor.
     * @return coral is interrupting the beam breaker.
     */
    public boolean coralEjecting() {
        return coralLoaded();
    }


    public void reverse(){
        setMotor(-0.2);
    }

    public int getProximity() {
        return colorSensor.getProximity();  // Returns 0 (far) to ~2047 (very close)
    }

    // coral detection from color sensor
    @AutoLogOutput(key = "Outtake/CoralLoaded")
    public boolean coralLoaded() {
        //this is about 1/2inch away -- might have to change based on placement
        return getProximity() > 800;
    }
}

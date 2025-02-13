package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
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

/**
 * Abstract class for the outtake. All commands should use this subsystem
 */
public abstract class Outtake extends SubsystemBase {


    /** Set the motor power to move the coral */
    public abstract void setMotor(double power);


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


    public abstract boolean coralLoaded();


    /**
     *  Coral is at the ejecting beam break sensor.
     * @return coral is interrupting the beam breaker.
     */
    public abstract boolean coralEjecting();


    public void reverse(){
        setMotor(-0.2);
    }
}

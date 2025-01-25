package frc.robot.subsystems.gpm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class Outtake extends SubsystemBase {

    private SparkFlex  motor = new SparkFlex(IdConstants.OUTTAKE_MOTOR, MotorType.kBrushless);
    private double power;

    /** Coral detected before the rollers */
    private DigitalInput digitalInputLoaded = new DigitalInput(9);
    /** Coral detected after the rollers */
    private DigitalInput digitalInputEjected = new DigitalInput(8);

    public Outtake(){
        motor.configure(new SparkFlexConfig()
            .inverted(true)
            .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    @Override
    public void periodic(){
        motor.set(power);
        SmartDashboard.putBoolean("Coral loaded", coralLoaded());
        SmartDashboard.putBoolean("Coral ejected", coralEjected());
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

    public boolean coralEjected(){
        return !digitalInputEjected.get();
    }
}

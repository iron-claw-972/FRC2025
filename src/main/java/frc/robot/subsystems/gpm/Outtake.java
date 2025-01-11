package frc.robot.subsystems.gpm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class Outtake extends SubsystemBase {

    private SparkFlex  motor = new SparkFlex(IdConstants.OUTTAKE_MOTOR, MotorType.kBrushless);
    public Outtake(){
        motor.configure(new SparkFlexConfig()
            .inverted(true)
            .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }
    public void setMotor(double power){
        motor.set(power);
    }
    public void stop(){
        setMotor(0);
    }
    public void outtake(){
        setMotor(0.25);
    }
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class Outtake extends SubsystemBase {

    private SparkFlex  motor = new SparkFlex(IdConstants.OUTTAKE_MOTOR, MotorType.kBrushless);
    private double power;
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
    }
    public void setMotor(double power){
        this.power = power;
    }
    public void stop(){
        setMotor(0);
    }
    public void outtake(){
        setMotor(SmartDashboard.getNumber("wheel speed", 0));
    }

    public void reverse(){
        setMotor(-0.2);
    }
}

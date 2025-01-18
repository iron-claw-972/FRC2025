package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakePrototype extends SubsystemBase {
    private TalonFX indexMotor;
    private TalonFX intakeMotor;
    private int indexMotorID = -1;
    private int intakeMotorID = -1;
    public GroundIntakePrototype(){
        indexMotor = new TalonFX(indexMotorID);
        intakeMotor = new TalonFX(intakeMotorID);
    }
    
    public void setIndexMotor(double speed){
        indexMotor.set(speed);
    }
    public void setIntakeMotor(double speed){
        intakeMotor.set(speed);
    }
    
}

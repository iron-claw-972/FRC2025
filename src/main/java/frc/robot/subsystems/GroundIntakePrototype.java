package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;


public class GroundIntakePrototype extends SubsystemBase {
    private TalonFX indexMotor;
    private TalonFX intakeMotor;
 
    public GroundIntakePrototype(){
        indexMotor = new TalonFX(IdConstants.indexMotorID);
        intakeMotor = new TalonFX(IdConstants.intakeMotorID);
    }
    
    public void setIndexMotor(double speed){
        indexMotor.set(speed);
    }
    public void setIntakeMotor(double speed){
        intakeMotor.set(speed);
    }
    public void setBothMotors(double speedIndex, double speedIntake ){
        indexMotor.set(speedIndex);
        intakeMotor.set(speedIntake);
    }
    
}

package frc.robot.subsystems.gpm;


import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class Outtake extends SubsystemBase {

    private CANSparkFlex  motor = new CANSparkFlex(IdConstants.OUTTAKE_MOTOR, MotorType.kBrushless);
    public Outtake(){
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(true);
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

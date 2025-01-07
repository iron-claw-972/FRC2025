package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

// TODO: Update based on design and motor type, possibly rename. This is currently for an outtake similar to the kitbot with a roller.
public class Outtake extends SubsystemBase {
    private TalonFX motor = new TalonFX(IdConstants.OUTTAKE_MOTOR);
    public Outtake(){
        motor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
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

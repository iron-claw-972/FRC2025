package frc.robot.constants;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

public class StorageIndexConstants {
    public static final int indexMotorID = 7;
    public static final int indexBeamBreak = 2; 
    public static final NeutralModeValue indexNeutralMode = NeutralModeValue.Brake;

    public static final int stallLimit = 25;

    public static final double intakeSpeed = 0.25;
    public static final double ejectShootSpeed = 0.4;
    public static final double ejectAmpFrontSpeed = 1.0;
    public static final double ejectTrapSpeed = 0.2;
     
    public static final double ejectShootTimeout = 0.5;
    public static final double ejectAmpFrontTimeout = 0.5;

    public static final IdleMode idleMode = IdleMode.kBrake;
}

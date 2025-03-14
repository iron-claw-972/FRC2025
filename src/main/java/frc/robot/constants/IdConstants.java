package frc.robot.constants;

import edu.wpi.first.wpilibj.I2C;

public class IdConstants {
    // Drivetrain
    public static final int DRIVE_FRONT_LEFT_ID = 1;
    public static final int STEER_FRONT_LEFT_ID = 2;
    public static final int ENCODER_FRONT_LEFT_ID = 3;
    public static final int DRIVE_FRONT_RIGHT_ID = 10;
    public static final int STEER_FRONT_RIGHT_ID = 11;
    public static final int ENCODER_FRONT_RIGHT_ID = 12;
    public static final int DRIVE_BACK_LEFT_ID = 7;
    public static final int STEER_BACK_LEFT_ID = 8;
    public static final int ENCODER_BACK_LEFT_ID = 9;
    public static final int DRIVE_BACK_RIGHT_ID = 4;
    public static final int STEER_BACK_RIGHT_ID = 5;
    public static final int ENCODER_BACK_RIGHT_ID = 6;
    public static final int PIGEON = 13;

    // LEDs
    public static final int CANDLE_ID = 1;

    // Elevator
    public static final int ELEVATOR_RIGHT_MOTOR = 50;
    public static final int ELEVATOR_BOTTOM_LIMIT_SWITCH = 29;
    public static final int ELEVATOR_TOP_LIMIT_SWITCH = 30;

	// Indexer
	public static final int INDEXER_MOTOR = 56;
	public static final int INDEXER_SENSOR = 24;

    // Climb
    public static final int CLIMB_MOTOR = 31;

    // Intake
    public static final int INTAKE_ROLLER = 51;
    public static final int INTAKE_PIVOT = 55; //55
    public static final int INTAKE_LASER_CAN = 25;

    // Outtake
    public static final int OUTTAKE_MOTOR_ALPHA = 14;
    public static final int OUTTAKE_MOTOR_COMP = 30; 
    public static final int OUTTAKE_DIO_EJECTING = 3;
    public static final I2C.Port i2cPort = I2C.Port.kMXP;

    //Arm
    public static final int ARM_MOTOR = 29;
    public static final int ARM_ABSOLUTE_ENCODER = 5;
}

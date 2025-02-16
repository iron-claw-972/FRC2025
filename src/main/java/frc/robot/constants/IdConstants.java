package frc.robot.constants;

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
    
    // Elevator
    public static final int ELEVATOR_LEFT_MOTOR = 20;
    public static final int ELEVATOR_RIGHT_MOTOR = 48;
    public static final int ELEVATOR_BOTTOM_LIMIT_SWITCH = 0;
    public static final int ELEVATOR_TOP_LIMIT_SWITCH = 21;

    // LEDs
    public static final int CANDLE_ID = 1;

    // Elevator
    public static final int ELEVATOR_LEFT_MOTOR = -1;
    public static final int ELEVATOR_RIGHT_MOTOR = -1;
    public static final int ELEVATOR_BOTTOM_LIMIT_SWITCH = 29;
    public static final int ELEVATOR_TOP_LIMIT_SWITCH = 30;

    // Add other subsystems here

	// Indexer
	public static final int INDEXER_MOTOR = 0;
	public static final int INDEXER_SENSOR = 29;

    // Climb
    public static final int CLIMB_MOTOR = 20;

    // Intake
    public static final int INTAKE_ROLLER = -1;
    public static final int INTAKE_PIVOT = 11;
    public static final int INTAKE_LASER_CAN = 55;

    //Outtake
     /** CAN ID for the Alphabot's outake */
     public static final int OUTTAKE_MOTOR_ALPHA = 14;
     /** CAN ID for the competition bot's outtake */
     public static final int OUTTAKE_MOTOR_COMP = -1; 
 
     // Digital inputs
     public static final int OUTTAKE_DIO_LOADED = 9;
     public static final int OUTTAKE_DIO_EJECTING = 8;
}

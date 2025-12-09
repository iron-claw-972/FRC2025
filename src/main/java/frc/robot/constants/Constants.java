package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

    // constants:   

    public static final double GRAVITY_ACCELERATION = 9.8;
    public static final double ROBOT_VOLTAGE = 12.0;
    public static final double LOOP_TIME = 0.02;

    // CAN bus names
    public static final String CANIVORE_CAN = "CANivore";
    public static final String RIO_CAN = "rio";

    // Logging 
    public static final boolean USE_TELEMETRY = true;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    // Kraken Speed
    public static double MAX_RPM = 5800.0; // Rotations per minute

    /*
     * Talon Stator / Supply Limits explanation
     * Supply current is current that's being drawn at the input bus voltage. Stator
     * current is current that's being drawn by the motor.
     * Supply limiting (supported by Talon FX and SRX) is useful for preventing
     * breakers from tripping in the PDP.
     * Stator limiting (supported by Talon FX) is useful for limiting
     * acceleration/heat.
     */

    // These are the default values

    // Stator
    public static final boolean TALONFX_STATOR_LIMIT_ENABLE = false; // enabled?
    public static final double TALONFX_STATOR_CURRENT_LIMIT = 100; // Limit(amp)
    public static final double TALONFX_STATOR_TRIGGER_THRESHOLD = 100; // Trigger Threshold(amp)
    public static final double TALONFX_STATOR_TRIGGER_DURATION = 0; // Trigger Threshold Time(s)

    // Supply
    public static final boolean TALONFX_SUPPLY_LIMIT_ENABLE = false; // enabled?
    public static final double TALONFX_SUPPLY_CURRENT_LIMIT = 40; // Limit(amp), current to hold after trigger hit
    public static final double TALONFX_SUPPLY_TRIGGER_THRESHOLD = 55; // (amp), amps to activate trigger
    public static final double TALONFX_SUPPLY_TRIGGER_DURATION = 3; // (s), how long after trigger before reducing

    // OIConstants:

    public static final int DRIVER_JOY = 0;
    public static final int OPERATOR_JOY = 1;
    public static final int TEST_JOY = 2;
    public static final int MANUAL_JOY = 3;
    public static final double DEFAULT_DEADBAND = 0.00005;

    public static final double TRANSLATIONAL_DEADBAND = 0.01;
    
    public static final double ROTATION_DEADBAND = 0.01;
    
    public static final double HEADING_DEADBAND = 0.05;
    public static final double HEADING_SLEWRATE = 10;

    //Modes
    public static final Mode SIM_MODE = Mode.REPLAY;
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    // Enables 3D logs of mechanisms
    public static final boolean LOG_MECHANISMS = true;
}

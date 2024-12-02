package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {

    // motor Ids
    public static final int RIGHT_MOTOR_ID = 6;
    public static final int LEFT_MOTOR_ID = 3;

    // RPMs
    public static final double LEFT_SHOOT_RPM = 6000;
    public static final double RIGHT_SHOOT_RPM = 4000;
    public static final double INTAKE_RPM = 0; // may not be neccessary if indexer spins fast enough
    public static final double EJECT_RPM = 0; // may not be neccessary if indexer spins fast enough
    public static final double AMP_OUTTAKE_RPM = 800; // may not be neccessary if indexer spins fast enough
    public static final double SHOOT_SPEED_MPS = 15;

    // The angle the shooter is at when the arm is at 0 degrees
    public static final double ANGLE_OFFSET = Units.degreesToRadians(43.58322);

    public static final double SPIN = -200;
}

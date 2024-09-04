package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class IntakeConstants {

    // motor port
    public static final int MOTOR_ID = 2;
    public static final int SENSOR_ID = 0;
    public static final int CENTERING_MOTOR_ID = 1;

    // intake speeds
    public static final double INTAKE_POWER = 0.8;
    public static final double CENTERING_POWER = 0.3;

    // Current limits
    public static final int CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int PEAK_CURRENT_LIMIT = 55;
    public static final double PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final double INTAKE_STALL_TIME = 0.2;
    public static final double INTAKE_CURRENT_STOP = 10;

    public static final IdleMode idleMode = IdleMode.kBrake;
    // MOI stuff
    public static final double MASS_SHAFT = 0.4; // in kilograms
    public static final double LENGTH_SHAFT = Units.inchesToMeters(25.5);
    public static final double MOI_SHAFT = (1.0 / 12.0) * MASS_SHAFT * LENGTH_SHAFT * LENGTH_SHAFT;
    public static final double MOI_TOTAL = MOI_SHAFT * 4;

    public static final double MASS_CENTERING_WHEELS = 0.1; // in kilograms
    public static final double RADIUS_CENTERING_WHEELS = Units.inchesToMeters(2);
    public static final double MOI_CENTERING_WHEEL = 0.5 * MASS_CENTERING_WHEELS * RADIUS_CENTERING_WHEELS
            * RADIUS_CENTERING_WHEELS;
    public static final double MOI_CENTERING_TOTAL = MOI_CENTERING_WHEEL * 4;

    //GearBoxes
    public static final DCMotor dcMotorCentering = DCMotor.getNEO(1);
    public static final DCMotor dcMotor = DCMotor.getNeoVortex(1);

    //Motor voltage
    public static final double motorVoltage = 12.0;
}
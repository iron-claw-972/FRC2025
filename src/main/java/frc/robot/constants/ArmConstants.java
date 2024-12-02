package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.util.Units;

public class ArmConstants {

    /** 
     * Arm motor ids.
     * <p>
     * left side motors: KR69(9), KR71(11);
     * <p>
     * right side motors: KR63(8), KR58(58)
     */
    public static final int[] MOTOR_IDS = new int[] {9, 11, 8, 58};

    /** The REV Duty Cycle encoder DIO channel */
    public static final int ENCODER_ID = 3;

    // TODO: use the real moment of inertia
    // guess the MOI as radius = 0.5 meter and the mass is 10 kg
    public static final double MOMENT_OF_INERTIA = 2.5;
    /** Arm length in meters */
    public static final double ARM_LENGTH = .5;

    /**
     * Minimum arm angle in radians.
     * <p>
     * This angle is really the stow or rest angle of the arm.
     * it was measured with a protractor.
    * 
     */
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(-8.8);
    /** maximum arm angle in radians */
    // TODO: jimmied to 73 so ampSetpoint will work
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(70.5 + 3.5);

    /** 
     * starting angle in radians.
     */
    public static final double START_ANGLE_RADS = MIN_ANGLE_RADS; // 9.2

    // If you add another setpoint field, check its validity in test/java/frc/robot/subsystems/gpm/ArmTest.java
    // TODO: update these values; e.g., stowedSetpoint should probably be MIN_ANGLE_RADS
    public static final double intakeSetpoint = MIN_ANGLE_RADS;
    public static final double stowedSetpoint = MIN_ANGLE_RADS;
    public static final double standbySetpoint = Units.degreesToRadians(40);
    public static final double subwooferSetpoint = MIN_ANGLE_RADS;
    public static final double preClimbSetpoint = MAX_ANGLE_RADS;
    public static final double climbSetpoint = MIN_ANGLE_RADS;
    public static final double zeroSetpoint = 0;
    public static double armFudgeFactor = 0; 
    public static double armFudgeFactorChanges = 0;

    // TODO: if the max angle is 70.5, then why is this being set to 73?
    public static final double ampSetpoint = Units.degreesToRadians(73.0);

    public static final double PIVOT_HEIGHT = Units.inchesToMeters(16.75);
    public static final double PIVOT_X = Units.inchesToMeters(-10);

	/** Whether to check if the arm is stowed on deploy */
	public static final boolean ASSERT_AT_SETPOINT = true;
	/** In radians */
	public static final double ANGLE_TOLERANCE = 0.2;

    public static CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(15)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentThreshold(20);
}

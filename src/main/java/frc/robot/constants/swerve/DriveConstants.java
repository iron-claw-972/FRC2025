package frc.robot.constants.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotId;
import frc.robot.constants.Constants;
import frc.robot.util.SwerveStuff.ModuleLimits;
import lib.COTSFalconSwerveConstants;

/**
 * Global constants are, by default, for the competition robot.
 * Global constants get changed in the update method if the RobotId detected is not the competition robot.
 */
public class DriveConstants {
    /**
     * The robot's width with its bumpers on.
     * <p>
     * The frame width is 26.5 inches, and each bumper is 3.25 inches.
     */
    public static final double ROBOT_WIDTH_WITH_BUMPERS = 0.832;

    public static double ROBOT_MASS = 25;

    /** Radius of the drive wheels [meters]. */
    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.95);

    public static double WHEEL_MOI = 0.000326 * ROBOT_MASS;

    
    /** Distance between the left and right wheels [meters]. */
    public static double TRACK_WIDTH = Units.inchesToMeters(20.75);//22.75 swerve bot, 20.75 comp bot

    // Mk4i gear ratios
    // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    //   standard gear ratios
    // https://www.swervedrivespecialties.com/products/kit-adapter-16t-drive-pinion-gear-mk4i
    //   changes 14-tooth pinion to 16-tooth pinion -- (50.0 / 14.0) becomes (50.0 / 16.0).
    /** Drive gear ratio for an Mk4i with L2-Plus gearing */
    public static double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
    // all MK4i modules have the same steering gear ratio
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    /** Theoretical maximum speed of the robot based on maximum motor RPM, gear ratio, and wheel radius */
    public static final double MAX_SPEED = 4.5;

    // Need to convert tangential velocity (the m/s of the edge of the robot) to angular velocity (the radians/s of the robot)
    // To do so, divide by the radius. The radius is the diagonal of the square chassis, diagonal = sqrt(2) * side_length.
    public static final double MAX_ANGULAR_SPEED = MAX_SPEED / ((TRACK_WIDTH / 2) * Math.sqrt(2));

    public static final double COSF = 0.9;
    
    // The maximum acceleration of the robot, limited by friction
    public static final double MAX_LINEAR_ACCEL = COSF * Constants.GRAVITY_ACCELERATION;
    // The maximum amount a drive motor can accelerate, independant of friction
    // This does nothing if greater than LINEAR_ACCEL
    public static final double MAX_DRIVE_ACCEL = MAX_LINEAR_ACCEL;
    // The maximum angular acceleration of the robot
    public static final double MAX_ANGULAR_ACCEL = MAX_LINEAR_ACCEL / TRACK_WIDTH * Math.sqrt(2);

    /**
     * If this is false, Drivetrain will use the previous setpoint to calculate the new setpoint.
     * <p> If this is true, Drivetrain will use the actual current setpoint instead.
     */
    public static final boolean USE_ACTUAL_SPEED = false;

    /**
     * Disables the deadband and optimization for the modules.
     * SwerveSetpointGenerator adds its own optimization and deadband, and the controllers also have a deadband.
     * Setting this to true fixes bugs caused by using hte actual current state.
     */
    public static final boolean DISABLE_DEADBAND_AND_OPTIMIZATION = false;

    public static final Rotation2d STARTING_HEADING = new Rotation2d();

    public static final Translation2d[] MODULE_LOCATIONS = {
        new Translation2d(DriveConstants.TRACK_WIDTH / 2, DriveConstants.TRACK_WIDTH / 2),
        new Translation2d(DriveConstants.TRACK_WIDTH / 2, -DriveConstants.TRACK_WIDTH / 2),
        new Translation2d(-DriveConstants.TRACK_WIDTH / 2, DriveConstants.TRACK_WIDTH / 2),
        new Translation2d(-DriveConstants.TRACK_WIDTH / 2, -DriveConstants.TRACK_WIDTH / 2)
    };

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_LOCATIONS);

    public static double STEER_OFFSET_FRONT_LEFT = 302.646;
    public static double STEER_OFFSET_FRONT_RIGHT = 103.039+180;
    public static double STEER_OFFSET_BACK_LEFT = 165.49+90;
    public static double STEER_OFFSET_BACK_RIGHT = 73.563;

    // Heading PID.
    public static final double HEADING_P = 5.5;
    public static final double HEADING_D = 0;

    public static final double HEADING_TOLERANCE = Units.degreesToRadians(1.5);

    // Translational PID
    // TODO: Tune this better (low priority since we aren't using it in 2025)
    public static final double TRANSLATIONAL_P = 1;
    public static final double TRANSLATIONAL_D = 0.001;

    //The PIDs for PathPlanner Command
    public static final double PATH_PLANNER_HEADING_P = 3.5;
    public static final double PATH_PLANNER_HEADING_D = 0;

    public static final double PATH_PLANNER_TRANSLATIONAL_P = 6;
    public static final double PATH_PLANNER_TRANSLATIONAL_D = 0;

    // CAN
    public static String DRIVE_MOTOR_CAN = Constants.CANIVORE_CAN;
    public static String STEER_MOTOR_CAN = Constants.CANIVORE_CAN;
    public static String STEER_ENCODER_CAN = Constants.CANIVORE_CAN;
    public static String PIGEON_CAN = Constants.CANIVORE_CAN;


    public static COTSFalconSwerveConstants MODULE_CONSTANTS = COTSFalconSwerveConstants.SDSMK4i(DRIVE_GEAR_RATIO);

    /* Swerve Current Limiting */
    public static final int STEER_CONTINUOUS_CURRENT_LIMIT = 15;
    public static final int STEER_PEAK_CURRENT_LIMIT = 15;
    public static final double STEER_PEAK_CURRENT_DURATION = 0.01;
    public static final boolean STEER_ENABLE_CURRENT_LIMIT = true;

    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 60;
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.01;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* Motor inversions */
    public static final InvertedValue INVERT_DRIVE_MOTOR = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue INVERT_STEER_MOTOR = InvertedValue.Clockwise_Positive;

    /* Neutral Modes */
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue STEER_NEUTRAL_MODE = NeutralModeValue.Brake;

    /* Drive Motor PID Values */
    public static final double[] P_VALUES = {
        0.1,
        0.1,
        0.1,
        0.1
    };
    public static final double[] I_VALUES = {
        0,
        0,
        0,
        0
    };
    public static final double[] D_VALUES = {
        0,
        0,
        0,
        0
    };
    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double[] S_VALUES = {
        0.11,
        0.11,
        0.11,
        0.11
    };
    public static final double[] V_VALUES = {
        0.11079,
        0.10718,
        0.11009,
        0.1164
    };
    public static final double[] A_VALUES = {
        0.005482,
        0.0049593,
        0.010156,
        0.0065708
    };
    /* Ramp values for drive motors in open loop driving. */
    // Open loop prevents throttle from changing too quickly.
    // It will limit it to time given (in seconds) to go from zero to full throttle.
    // A small open loop ramp (0.25) helps with tread wear, tipping, etc
    public static final double OPEN_LOOP_RAMP = 0.25;

    public static final double WHEEL_CIRCUMFERENCE = 2*Math.PI*WHEEL_RADIUS;

    public static final boolean INVERT_GYRO = false; // Make sure gyro is CCW+ CW-

    public static final double SLOW_DRIVE_FACTOR = 0.2;
    public static final double SLOW_ROT_FACTOR = 0.1;

    public static final ModuleLimits MODULE_LIMITS = new ModuleLimits(MAX_SPEED, MAX_DRIVE_ACCEL, COSF, Units.rotationsPerMinuteToRadiansPerSecond(Constants.MAX_RPM / STEER_GEAR_RATIO));

    /**
     * Updates the constants if the RobotId is not the competition robot.
     */
    public static void update(RobotId robotId) {
        if(robotId == RobotId.BetaBot) {
            STEER_OFFSET_FRONT_LEFT = 193.884-180;
            STEER_OFFSET_FRONT_RIGHT = 110.914;
            STEER_OFFSET_BACK_LEFT = 128.054+180;
            STEER_OFFSET_BACK_RIGHT = 107.43;
        } else if (robotId == RobotId.Vivace) {
            STEER_OFFSET_FRONT_LEFT = 100.184+180;
            STEER_OFFSET_FRONT_RIGHT = 224.293;
            STEER_OFFSET_BACK_LEFT = 304.795-180;
            STEER_OFFSET_BACK_RIGHT = 201.177-180;

            ROBOT_MASS = 50;
            WHEEL_MOI = 0.000326 * ROBOT_MASS;

        } else if (robotId == RobotId.Vertigo) {
            STEER_OFFSET_FRONT_LEFT = 4.185;
            STEER_OFFSET_FRONT_RIGHT = 101.519+90;
            STEER_OFFSET_BACK_LEFT = 38.997+180;
            STEER_OFFSET_BACK_RIGHT = 242.847-90;
            
            DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

            ROBOT_MASS = 20;

            WHEEL_MOI = 0.000326 * ROBOT_MASS;
            
            // Falcon Speed
            Constants.MAX_RPM = 6080.0;
        } else if (robotId == RobotId.Phil) {
            ROBOT_MASS = 30;
            WHEEL_MOI = 0.000326 * ROBOT_MASS;

            STEER_OFFSET_FRONT_LEFT = 121.463+180;
            STEER_OFFSET_FRONT_RIGHT = 284.242;
            STEER_OFFSET_BACK_LEFT = 157.676;
            STEER_OFFSET_BACK_RIGHT = 77.199;

            DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        }
        
        MODULE_CONSTANTS = COTSFalconSwerveConstants.SDSMK4i(DRIVE_GEAR_RATIO);
    }
}

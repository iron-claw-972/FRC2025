package frc.robot.constants.swerve;

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
    public static final double ROBOT_WIDTH_WITH_BUMPERS = Units.inchesToMeters(26.5 + 3.25 * 2);

    /** Radius of the drive wheels [meters]. */
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2);

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
    public static final double MAX_SPEED = (Constants.MAX_RPM / 60.0) * WHEEL_RADIUS * 2 * Math.PI / DRIVE_GEAR_RATIO;

    // Need to convert tangential velocity (the m/s of the edge of the robot) to angular velocity (the radians/s of the robot)
    // To do so, divide by the radius. The radius is the diagonal of the square chassis, diagonal = sqrt(2) * side_length.
    public static final double MAX_ANGLULAR_SPEED = MAX_SPEED / ((TRACK_WIDTH / 2) * Math.sqrt(2));

    public static final double COSF = 1.1;
    
    public static double MAX_LINEAR_ACCEL = COSF * Constants.GRAVITY_ACCELERATION;
    public static double MAX_ANGULAR_ACCEL = MAX_LINEAR_ACCEL * kTrackWidth * Math.sqrt(2) / 2;

    public static final Rotation2d STARTING_HEADING = new Rotation2d();

    public static final Translation2d[] MODULE_LOCATIONS = {
        new Translation2d(DriveConstants.TRACK_WIDTH / 2, DriveConstants.TRACK_WIDTH / 2),
        new Translation2d(DriveConstants.TRACK_WIDTH / 2, -DriveConstants.TRACK_WIDTH / 2),
        new Translation2d(-DriveConstants.TRACK_WIDTH / 2, DriveConstants.TRACK_WIDTH / 2),
        new Translation2d(-DriveConstants.TRACK_WIDTH / 2, -DriveConstants.TRACK_WIDTH / 2)
    };

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_LOCATIONS);

    public static double STEER_OFFSET_FRONT_LEFT = 0;
    public static double STEER_OFFSET_FRONT_RIGHT = 0;
    public static double STEER_OFFSET_BACK_LEFT = 0;
    public static double STEER_OFFSET_BACK_RIGHT = 0;

    // Heading PID.
    public static final double HEADING_P = 5.5;
    public static final double HEADING_D = 0;

    public static final double HEADING_TOLERANCE = Units.degreesToRadians(1.5);

    // Translational PID
    // TODO: Tune this better
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

    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 80;
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 80;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.01;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* Motor inversions */
    public static final boolean INVERT_DRIVE_MOTOR = true;
    public static final boolean INVERT_STEER_MOTOR = MODULE_CONSTANTS.angleMotorInvert;

    /* Neutral Modes */
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue STEER_NEUTRAL_MODE = NeutralModeValue.Brake;

    /* Drive Motor PID Values */
    public static final double[] P_VALUES = {
        0.035524,
        0.075025,
        0.1088,
        0.085856
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
        0.11079,
        0.1117,
        0.11257,
        0.075667
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

    public static final ModuleLimits MODULE_LIMITS = new ModuleLimits(MAX_SPEED, MAX_LINEAR_ACCEL, Units.rotationsPerMinuteToRadiansPerSecond(Constants.MAX_RPM / STEER_GEAR_RATIO));

    /**
     * Updates the constants if the RobotId is not the competition robot.
     */
    public static void update(RobotId robotId) {
        if (robotId == RobotId.Vivace) {
            STEER_OFFSET_FRONT_LEFT = 100.184;
            STEER_OFFSET_FRONT_RIGHT = 224.293-180;
            STEER_OFFSET_BACK_LEFT = 304.795;
            STEER_OFFSET_BACK_RIGHT = 201.177;
        } else if (robotId == RobotId.Vertigo) {
            TRACK_WIDTH = Units.inchesToMeters(22.75);//22.75 swerve bot, 20.75 comp bot
            
            STEER_OFFSET_FRONT_LEFT = 4.185;
            STEER_OFFSET_FRONT_RIGHT = 101.519+90;
            STEER_OFFSET_BACK_LEFT = 38.997+180;
            STEER_OFFSET_BACK_RIGHT = 242.847-90;
            
            DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
            
            // Falcon Speed
            Constants.MAX_RPM = 6080.0;
        } else if (robotId == RobotId.Phil) {

            DRIVE_MOTOR_CAN = Constants.RIO_CAN;
            STEER_MOTOR_CAN = Constants.RIO_CAN;
            STEER_ENCODER_CAN = Constants.RIO_CAN;
            PIGEON_CAN = Constants.RIO_CAN;

            TRACK_WIDTH = Units.inchesToMeters(22.75); //22.75 swerve bot, 20.75 comp bot
        
            STEER_OFFSET_FRONT_LEFT = 121.463;
            STEER_OFFSET_FRONT_RIGHT = 284.242-180;
            STEER_OFFSET_BACK_LEFT = 157.676+180;
            STEER_OFFSET_BACK_RIGHT = 77.199+180;

            DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        }
        MODULE_CONSTANTS = COTSFalconSwerveConstants.SDSMK4i(DRIVE_GEAR_RATIO);
    }
}

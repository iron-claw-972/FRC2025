package frc.robot.constants.swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotId;
import frc.robot.constants.Constants;
import lib.COTSFalconSwerveConstants;

/**
 * GlobalConst are, by default, for the competition robot.
 * GlobalConst get changed if the RobotId detected is not the competition robot.
 */
public class DriveConstants {
    /**
     * The robot's width with its bumpers on.
     * <p>
     * The frame width is 26.5 inches, and each bumper is 3.25 inches.
     */
    public static double kRobotWidthWithBumpers = Units.inchesToMeters(26.5 + 3.25 * 2);

    // TODO: missing robot length
    // TODO: missing robot center of rotation to back bumper and center of rotation to front bumper
    // the center of rotation need not be the geometric center of the robot.

    /** Radius of the drive wheels [meters]. */
    public static double kWheelRadius = Units.inchesToMeters(2);

    /** Distance between the left and right wheels [meters]. */
    public static double kTrackWidth = Units.inchesToMeters(20.75);//22.75 swerve bot, 20.75 comp bot

    // Mk4i gear ratios
    // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    //   standard gear ratios
    // https://www.swervedrivespecialties.com/products/kit-adapter-16t-drive-pinion-gear-mk4i
    //   changes 14-tooth pinion to 16-tooth pinion -- (50.0 / 14.0) becomes (50.0 / 16.0).
    /** Drive gear ratio for an Mk4i with L2-Plus gearing */
    public static double kDriveGearRatio = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
    // all MK4i modules have the same steering gear ratio
    public static double kSteerGearRatio = 150.0 / 7.0;

    /** Theoretical maximum speed of the robot based on maximum motor RPM, gear ratio, and wheel radius */
    public static double kMaxSpeed = (Constants.MAX_RPM / 60.0) * kWheelRadius * 2 * Math.PI / kDriveGearRatio;

    // Need to convert tangential velocity (the m/s of the edge of the robot) to angular velocity (the radians/s of the robot)
    // To do so, divide by the radius. The radius is the diagonal of the square chassis, diagonal = sqrt(2) * side_length.
    public static double kMaxAngularSpeed = kMaxSpeed / ((kTrackWidth / 2) * Math.sqrt(2));

    // TODO: tune this better.
    public static double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

    /** Pigeon2 IMU CAN Id. */
    public static int kPigeon = 13;

    public static Rotation2d kStartingHeading = new Rotation2d();

    public static final Translation2d[] swerveModuleLocations = {
            new Translation2d(DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2),
            new Translation2d(DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2),
            new Translation2d(-DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2),
            new Translation2d(-DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2)
    };

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(swerveModuleLocations);

    public static int kDriveFrontLeft = 1;
    public static int kSteerFrontLeft = 2;
    public static int kEncoderFrontLeft = 3;
    public static double kSteerOffsetFrontLeft = 100.184;//0.058291152119637;//-3.060285486280918+Math.PI;

    public static int kDriveFrontRight = 10;
    public static int kSteerFrontRight = 11;
    public static int kEncoderFrontRight = 12;
    public static double kSteerOffsetFrontRight = 224.293-180;//-2.994324445724487;//-3.001994334161282;

    public static int kDriveBackLeft = 7;
    public static int kSteerBackLeft = 8;
    public static int kEncoderBackLeft = 9;
    public static double kSteerOffsetBackLeft =304.795;//-2.540267050266266;//0.650406539440155+Math.PI;

    public static int kDriveBackRight = 4;
    public static int kSteerBackRight = 5;
    public static int kEncoderBackRight = 6;
    public static double kSteerOffsetBackRight = 201.177;//2.626169800758362;//2.771897681057453;

    // heading PID
    public static double kHeadingP = 5.5;
    public static double kHeadingD = 0;

    public static final double HEADING_TOLERANCE = Units.degreesToRadians(1.5);

    //translational PID
    public static double kTranslationalP = 1;
    public static double kTranslationalD = 0.001;

    //The PIDs for PathPlanner Command
    public static double kPathplannerHeadingP = 3.5;
    public static double kPathplannerHeadingD = 0;

    public static double kPathplannerTranslationalP = 6;
    public static double kPathplannerTranslationalD = 0;

    // CAN
    public static String kDriveMotorCAN = Constants.CANIVORE_CAN;
    public static String kSteerMotorCAN = Constants.CANIVORE_CAN;
    public static String kSteerEncoderCAN = Constants.CANIVORE_CAN;
    public static String kPigeonCAN = Constants.CANIVORE_CAN;


    public static COTSFalconSwerveConstants kModuleConstants = COTSFalconSwerveConstants.SDSMK4i(kDriveGearRatio);

    /* Swerve Current Limiting */
    public static final int kAngleContinuousCurrentLimit = 15;
    public static final int kAnglePeakCurrentLimit = 15;
    public static final double kAnglePeakCurrentDuration = 0.01;
    public static final boolean kAngleEnableCurrentLimit = true;

    public static final int kDriveContinuousCurrentLimit = 80;
    public static final int kDrivePeakCurrentLimit = 80;
    public static final double kDrivePeakCurrentDuration = 0.01;
    public static final boolean kDriveEnableCurrentLimit = true;

    // TODO put slew rate limiter to reduce drift on drivetrain without killing battery.

    /* Motor inversions */
    public static final boolean kDriveMotorInvert = true;//kModuleConstants.driveMotorInvert;
    public static final boolean kAngleMotorInvert = kModuleConstants.angleMotorInvert;

    /* Neutral Modes */
    public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue kAngleNeutralMode = NeutralModeValue.Brake;

    /* Drive Motor PID Values */
    public static final double[] P_Values= {
        0.035524,
        0.075025,
        0.1088,
        0.085856
        
    };
    public static final double[] I_Values= {
        0,
        0,
        0,
        0
    };
    public static final double[] D_Values= {
        0,
        0,
        0,
        0
    };
    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double[] DRIVE_KS_Values= {
        0.11079,
        0.1117,
        0.11257,
        0.075667
    };
    public static final double[] DRIVE_KV_Values= {
        0.11079,
        0.10718,
        0.11009,
        0.1164
    };
    public static final double[] DRIVE_KA_Values= {
        0.005482,
        0.0049593,
        0.010156,
        0.0065708
    };
    /* Ramp values for drive motors in open and closed loop driving. */
    // Open loop prevents throttle from changing too quickly.
    // It will limit it to time given (in seconds) to go from zero to full throttle.
    // A small open loop ramp (0.25) helps with tread wear, tipping, etc
    public static final double kOpenLoopRamp = 0.25;
    public static final double kClosedLoopRamp = 0.0;

    public static final double kWheelCircumference = kModuleConstants.wheelCircumference;

    /* Motor gear ratios */
    public static final double kAngleGearRatio = kModuleConstants.angleGearRatio;

    public static final boolean kInvertGyro = false; // Make sure gyro is CCW+ CW- // FIXME: Swerve

    public static final double kSlowDriveFactor = 0.2;
    public static final double kSlowRotFactor = 0.1;

    /**
     * Updates the constants if the RobotId is not the competition robot.
     */
    public static void update(RobotId robotId) {
        if (robotId == RobotId.Vertigo) {
            kTrackWidth = Units.inchesToMeters(22.75);//22.75 swerve bot, 20.75 comp bot
            
            kPigeon = 13;

            kSteerOffsetFrontLeft = 4.185;//0.058291152119637;//-3.060285486280918+Math.PI;

            kSteerOffsetFrontRight = 101.519+90;//-2.994324445724487;//-3.001994334161282;

            kSteerOffsetBackLeft = 38.997+180;//-2.540267050266266;//0.650406539440155+Math.PI;

            kSteerOffsetBackRight = 242.847-90;//2.626169800758362;//2.771897681057453;
            
            kDriveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
            kModuleConstants = COTSFalconSwerveConstants.SDSMK4i(kDriveGearRatio);
            
            // Talon Speed
            Constants.MAX_RPM = 6080.0;
        } 
        else if (robotId == RobotId.SwerveTest) {

            kTrackWidth = Units.inchesToMeters(22.75); //22.75 swerve bot, 20.75 comp bot

            kPigeon = 13;
        
            kSteerOffsetFrontLeft = -448.91;

            kSteerOffsetFrontRight = 112.473;
            // kSteerOffsetFrontRight = 10.957+90;

            kSteerOffsetBackLeft = 180;
            // [new one] kSteerOffsetBackLeft = 339.689;

            kSteerOffsetBackRight = 333.241;
            // Talon Speed
            Constants.MAX_RPM = 6080.0;

            kDriveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
            kModuleConstants = COTSFalconSwerveConstants.SDSMK4i(kDriveGearRatio);
        }
    }
}

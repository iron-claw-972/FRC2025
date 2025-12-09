package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.swerve.DriveConstants;

/**
 * Container class for auto constants.
 */
public class AutoConstants {

    // Pathplanner output folder should be src/main/deploy/pathplanner
    public static final String TRAJECTORY_DIRECTORY = "pathplanner/paths/";

    public static final double MAX_AUTO_SPEED = 5.2; // m/s
    public static final double MAX_AUTO_ACCEL = 4.8; // m/s^2

    public static RobotConfig  CONFIG;
    public static final PPHolonomicDriveController AUTO_CONTROLLER = new PPHolonomicDriveController( 
            new PIDConstants(4.0, 0.0, 0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        );
   

        
    static {
        try{
            CONFIG = RobotConfig.fromGUISettings();
        }catch(Exception e){
            e.printStackTrace();
            // Although these values are probably wrong and auto might not work correctly, at least it won't cause NullPointerExceptions
            CONFIG = new RobotConfig(50, 0.5, new ModuleConfig(DriveConstants.WHEEL_RADIUS, MAX_AUTO_SPEED, DriveConstants.COSF, DCMotor.getKrakenX60(1).withReduction(DriveConstants.DRIVE_GEAR_RATIO), DriveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT, 1), DriveConstants.MODULE_LOCATIONS);
        }
    }
}

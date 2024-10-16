package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.constants.swerve.DriveConstants;

/**
 * Container class for auto constants.
 */
public class AutoConstants {

    // Pathplanner output folder should be src/main/deploy/pathplanner
    public static final String TRAJECTORY_DIRECTORY = "pathplanner/paths/";

    public static final double MAX_AUTO_SPEED = 5.2; // m/s
    public static final double MAX_AUTO_ACCEL = 4.8; // m/s^2

    public static RobotConfig  config;
    public static final PPHolonomicDriveController autoController = new PPHolonomicDriveController( 
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            );

   

    private AutoConstants() {
        try{
            config = RobotConfig.fromGUISettings();
        }
        catch(Exception e){
            e.printStackTrace();
        }
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.util.LogManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autoCommand;
    private RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // To Set the Robot Identity
        //   SimGUI: Persistent Values, Preferences, RobotId, then restart Simulation
        //     changes networktables.json, networktables.json.bck (both Untracked)
        //   Uncomment the next line, set the desired RobotId, deploy, and then comment the line out
        //RobotId.setRobotId(RobotId.Vertigo);
        DriveConstants.update(RobotId.getRobotId());
        RobotController.setBrownoutVoltage(6);
        // obtain this robot's identity
        RobotId robotId = RobotId.getRobotId();

        // build the RobotContainer with the robot id from preferences
        robotContainer = new RobotContainer(robotId);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode-specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.

        // TODO: why is this here?
        robotContainer.updateShuffleBoard();

        CommandScheduler.getInstance().run();
        
        LogManager.update();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically when the robot is disabled
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // Disable vision if the constant is false.
        robotContainer.setVisionEnabled(VisionConstants.ENABLED_AUTO);

        // Get the autonomous command.
        // This access is fast (about 14 microseconds) because the value is already resident in the Network Tables.
        // There was a problem last year because the operation also installed about over a dozen items (taking more than 20 ms).
        autoCommand = robotContainer.getAutonomousCommand();

        // If there is an autonomous command, then schedule it
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }


    /**
     * This function is called once each time the robot enters Teleop mode.
     */
    @Override
    public void teleopInit() {
        // TODO: why is this here? Robot may not have vision.
        robotContainer.setVisionEnabled(true);

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * This function is called once each time the robot enters Test mode.
     */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

	/**
	* Gets the set Alliance; defaults to red if not set.
	* This method replaces {@link edu.first.wpilibj.DriverStation.getAlliance}.
	* The .get() is not necessary, so DriverStation.getAlliance().get() becomes Robot.getAlliance()
	*/
	public static Alliance getAlliance() {
		Optional<Alliance> dsAlliance = DriverStation.getAlliance();
		if (dsAlliance.isPresent())
			return dsAlliance.get();
		else
			return Alliance.Red; // default to Red alliance
	}
}

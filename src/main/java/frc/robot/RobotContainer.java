package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Turret;
import frc.robot.util.DetectedObject;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.ShuffleBoard.ShuffleBoardManager;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;
import frc.robot.util.Vision;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems are defined here...
  private Drivetrain drive = null;
  private Vision vision = null;
  private Turret turret = null;  

  // Controllers are defined here
  private BaseDriverConfig driver = null;
  private Operator operator = null;
  ShuffleBoardManager shuffleboardManager = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * <p>
   * Different robots may have different subsystems.
   */
  public RobotContainer(RobotId robotId) {
    turret = new Turret();

    SmartDashboard.putData("0", new InstantCommand(() -> turret.setAngle(0)));
    SmartDashboard.putData("90", new InstantCommand(() -> turret.setAngle(90)));
    SmartDashboard.putData("180", new InstantCommand(() -> turret.setAngle(180)));
    SmartDashboard.putData("270", new InstantCommand(() -> turret.setAngle(270)));

    SmartDashboard.putData("180 at 5 seconds", new InstantCommand(() -> turret.setAngleWithTime(180, 5)));
    SmartDashboard.putData("0 at 20 seconds", new InstantCommand(() -> turret.setAngleWithTime(0, 20)));
    SmartDashboard.putData("180 at 1 seconds", new InstantCommand(() -> turret.setAngleWithTime(180, 1)));
    SmartDashboard.putData("90 at 8 seconds", new InstantCommand(() -> turret.setAngleWithTime(90, 8)));
    SmartDashboard.putData("0 at 0.1 seconds", new InstantCommand(() -> turret.setAngleWithTime(0, 0.1)));
    SmartDashboard.putData("270 at 2 seconds", new InstantCommand(() -> turret.setAngleWithTime(270, 2)));
    
    SmartDashboard.putNumber("Angle To Turn", 0);
    SmartDashboard.putData("Turn to input angle", new InstantCommand(() -> turret.setAngle(SmartDashboard.getNumber("Angle To Turn", 0))));
    SmartDashboard.putData("Toggle Time Constrain", new InstantCommand(() -> turret.switchTimeConstraint()));
    switch (robotId) {

      case TestBed1:
        break;

      case TestBed2:
        break;

      default:
      case SwerveCompetition:
      // Our competition subsystems go here

      case Vivace:
        vision = new Vision(VisionConstants.APRIL_TAG_CAMERAS);
 
      case Phil:
      case Vertigo:
        drive = new Drivetrain(vision);
        driver = new GameControllerDriverConfig(drive, vision);
        operator = new Operator(drive);

        // Detected objects need access to the drivetrain
        DetectedObject.setDrive(drive);
        
        //SignalLogger.start();

        driver.configureControls();
        operator.configureControls();
        initializeAutoBuilder();
        registerCommands();
        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
        PathGroupLoader.loadPathGroups();
 
        shuffleboardManager = new ShuffleBoardManager(drive, vision);
        break;
      }

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // TODO: verify this claim.
    // LiveWindow is causing periodic loop overruns
    LiveWindow.disableAllTelemetry();
    LiveWindow.setEnabled(false);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command pathCommand = shuffleboardManager.getSelectedCommand();
    return pathCommand;
  }

  public void updateShuffleBoard() {
    if (shuffleboardManager != null)
      shuffleboardManager.update();
  }

  /**
   * Sets whether the drivetrain uses vision to update odometry
   */
  public void setVisionEnabled(boolean enabled) {
    if (drive != null)
      drive.setVisionEnabled(enabled);
  }

  public void initializeAutoBuilder() {
    AutoBuilder.configure(
        () -> drive.getPose(),
        (pose) -> {
          drive.resetOdometry(pose);
        },
        () -> drive.getChassisSpeeds(),
        (chassisSpeeds) -> {
          drive.setChassisSpeeds(chassisSpeeds, false); // problem??
        },
        AutoConstants.AUTO_CONTROLLER,
        AutoConstants.CONFIG,
        getAllianceColorBooleanSupplier(),
        drive);
  }

  public void registerCommands() {
  }

  public static BooleanSupplier getAllianceColorBooleanSupplier() {
    return () -> {
      // Boolean supplier that controls when the path will be mirrored for the red
      // alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };
  }
}



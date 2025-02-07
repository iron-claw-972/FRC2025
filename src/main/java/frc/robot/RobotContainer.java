package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.controls.PS5ControllerDriverConfig;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.PowerPanel;
import frc.robot.util.DetectedObject;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.ShuffleBoard.ShuffleBoardManager;
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
  private Intake intake = null;
  private Indexer indexer = null;
  private Outtake outtake = null;
  private Elevator elevator = null;
  private Climb climb = null;
  // all robots should have a power distribution panel or hub
  public static PowerPanel pdh;

  // Controllers are defined here
  private BaseDriverConfig driver = null;
  private Operator operator = null;
  private ShuffleBoardManager shuffleboardManager = null;

  private Thread odometryThread = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * <p>
   * Different robots may have different subsystems.
   */
  public RobotContainer(RobotId robotId) {
    // all robots have a power panel
    if (pdh == null) {
      pdh = new PowerPanel();
    }

    // dispatch on the robot
    switch (robotId) {

      case TestBed1:
        break;

      case TestBed2:
        break;

      default:
      case SwerveCompetition:
        // Our competition subsystems go here
        intake = new Intake();
        indexer = new Indexer();
        outtake = new Outtake();
        elevator = new Elevator();
        climb = new Climb();
        vision = new Vision(VisionConstants.APRIL_TAG_CAMERAS);

      case Vivace:
      case Phil:
      case Vertigo:
        drive = new Drivetrain(vision);
        driver = new PS5ControllerDriverConfig(drive, elevator, intake, indexer, outtake, climb);
        operator = new Operator(drive, elevator, intake, indexer, outtake, climb);

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
    
    // Start a new thread to update the odometry
    if(drive != null){
      odometryThread = new Thread(()->{
        while(!odometryThread.isInterrupted()){
          drive.updateOdometry();
        }
      });
      odometryThread.start();
    }
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

  public void interruptOdometryThraed(){
    odometryThread.interrupt();
  }

  /**
   * Get the PowerPanel subsystem.
   * <p>
   * Obtain the PowerPanel.
   * This method avoids passing in the PowerPanel to every substem because the PowerPanel is only needed for simulation.
   * The PowerPanel subsystem can be used to get the battery voltage an to get or set channel currents.
   * @return PowerPanel subsystem
   */
  public static PowerPanel getPowerPanel() {
    return pdh;
  }
}




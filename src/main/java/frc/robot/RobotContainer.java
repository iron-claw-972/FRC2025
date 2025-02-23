package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.gpm.OutakeMotors;
import frc.robot.constants.AutoConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.controls.PS5ControllerDriverConfig;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.OuttakeAlpha;
import frc.robot.subsystems.OuttakeComp;
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

  // Controllers are defined here
  private BaseDriverConfig driver = null;
  private Operator operator = null;
  private ShuffleBoardManager shuffleboardManager = null;

  private Thread odometryThread = null;
  private Thread drivetrainThread = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * <p>
   * Different robots may have different subsystems.
   */
  public RobotContainer(RobotId robotId) {

    // dispatch on the robot
    switch (robotId) {

      case TestBed1:
        break;

      case TestBed2:
        break;

      default:
      case SwerveCompetition:
        //  outtake = new OuttakeComp();
         //elevator = new Elevator();
        // climb = new Climb();
      case BetaBot:
        indexer = new Indexer();
        intake = new Intake();
        //SmartDashboard.putData("commadn schedule", CommandScheduler.getInstance());
        //vision = new Vision(VisionConstants.APRIL_TAG_CAMERAS);
        // fall-through
        //  SmartDashboard.putData("RunIntakeAndIndexer", new RunIntakeAndIndexer(intake, indexer));

      case Vivace:
      case Phil:
        if (robotId == RobotId.Phil) {
          outtake = new OuttakeAlpha();
        }
        if (outtake != null) {
          //SmartDashboard.putData("OuttakeCoralBasic", new OutakeMotors(intake, outtake));
          SmartDashboard.putData("OuttakeCoralBasic", new OutakeMotors(intake, outtake));
        }
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
 
        //shuffleboardManager = new ShuffleBoardManager(drive, vision);
      
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
      drivetrainThread = new Thread(()->{
        long nextUpdate = System.currentTimeMillis();
        while(!drivetrainThread.isInterrupted()){
          if(System.currentTimeMillis() >= nextUpdate){
            drive.drive(driver);
            nextUpdate += 20;
          }
        }
      });
      odometryThread.setPriority(3);
      drivetrainThread.setPriority(4);
      odometryThread.start();
      drivetrainThread.start();
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
   * Sets whether the drivetrain uses vision toupdate odometry
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

  public void interruptThreads(){
    odometryThread.interrupt();
    drivetrainThread.interrupt();
  }
}



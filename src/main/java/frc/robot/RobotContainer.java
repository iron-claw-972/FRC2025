package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.controls.PS5ControllerDriverConfig;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.sim.SimClimb;
import frc.robot.subsystems.sim.SimDrivetrain;
import frc.robot.subsystems.sim.SimElevator;
import frc.robot.subsystems.sim.SimIndexer;
import frc.robot.subsystems.sim.SimIntake;
import frc.robot.subsystems.sim.SimOuttake;
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * <p>
   * Different robots may have different subsystems.
   */
  public RobotContainer(RobotId robotId) {
    // Our normal switch statement doesn't work because each case continues to the next one, possibly creating duplicate sim subsystems

    // 2 robots have an elevator, outtake, and vision
    if(robotId == RobotId.Phil || robotId == RobotId.SwerveCompetition){
      elevator = new Elevator();
      outtake = new Outtake();
      vision = new Vision(VisionConstants.APRIL_TAG_CAMERAS);
    }else{
      elevator = new SimElevator();
      outtake = new SimOuttake();
      // Vision doesn't have a sim version
    }

    // Only the competition robot has the rest of the subsystems
    if(robotId == RobotId.SwerveCompetition){
      intake = new Intake();
      indexer = new Indexer();
      climb = new Climb();
    }else{
      intake = new SimIntake();
      indexer = new SimIndexer();
      climb = new SimClimb();
    }
    
    // if(robotId == RobotId.Phil){
    //   driver = new PS5ControllerDriverConfig(drive, elevator, intake, indexer, outtake, climb);
    // }

    // All of these robots need a drivetrain
    if(robotId == RobotId.SwerveCompetition || robotId == RobotId.Phil || robotId == RobotId.Vertigo || robotId == RobotId.Vivace){
      drive = new Drivetrain(vision);
    }else{
      drive = new SimDrivetrain(vision);
    }

    // All robots need controllers
    // Check the controller type to prevent it from breaking
    
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

    shuffleboardManager = new ShuffleBoardManager(drive, vision, elevator, outtake);

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // TODO: verify this claim.
    // LiveWindow is causing periodic loop overruns
    LiveWindow.disableAllTelemetry();
    LiveWindow.setEnabled(false);
    
    // Start a new thread to update the odometry
    odometryThread = new Thread(()->{
      while(!odometryThread.isInterrupted()){
        drive.updateOdometry();
      }
    });
    odometryThread.start();
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


    if(elevator != null && outtake != null){

      NamedCommands.registerCommand("Outtake_L4", new OuttakeCoral(outtake, elevator).withTimeout(1.5));


      NamedCommands.registerCommand("Intake", new SequentialCommandGroup(
        new MoveElevator(elevator, ElevatorConstants.INTAKE_SETPOINT),
        new WaitCommand(1),
        new InstantCommand(()->elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT))
      ));
      NamedCommands.registerCommand("Score L2", new SequentialCommandGroup(
        new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT),
        new OuttakeCoral(outtake, elevator)
      ));
      NamedCommands.registerCommand("Score L3", new SequentialCommandGroup(
        new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT),
        new OuttakeCoral(outtake, elevator)
      ));
      NamedCommands.registerCommand("Score L4", new SequentialCommandGroup(
        new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT),
        new OuttakeCoral(outtake, elevator)
      ));
    }
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

  public void interruptOdometryThread(){
    odometryThread.interrupt();
  }
}



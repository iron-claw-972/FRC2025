package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.gpm.IntakeNote;
import frc.robot.commands.gpm.PrepareShooter;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.ShuffleBoard.ShuffleBoardManager;
import frc.robot.util.Vision;
import lib.controllers.GameController.RumbleStatus;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

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
  private Arm arm = null;
  private Shooter shooter = null;
  private Intake intake = null;
  private StorageIndex index = null;

  // Controllers are defined here
  private BaseDriverConfig driver = null;
  private Operator operator =null;
  ShuffleBoardManager shuffleboardManager = null;

  Consumer<Boolean> consumer = bool -> {
    if (bool){
        operator.getGameController().setRumble(RumbleStatus.RUMBLE_ON);
      ((GameControllerDriverConfig) driver).getGameController().setRumble(RumbleStatus.RUMBLE_ON);
    }
    else{
        operator.getGameController().setRumble(RumbleStatus.RUMBLE_OFF);
        ((GameControllerDriverConfig) driver).getGameController().setRumble(RumbleStatus.RUMBLE_OFF);
    }
};

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * <p>
   * Different robots may have different subsystems.
   */
  public RobotContainer(RobotId robotId) {
    // dispatch on the robot
    switch (robotId) {

      case TestBed1:
        index = new StorageIndex();
        shooter = new Shooter();
        break;

      case TestBed2:
        intake = new Intake();
        index = new StorageIndex();
        break;
      case Vertigo:
          drive = new Drivetrain(vision);
          driver = new GameControllerDriverConfig(drive, vision, arm, intake, index, shooter);
          driver.configureControls();
          drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
          break;
        
      default:
      case SwerveCompetition:
        arm = new Arm();
        intake = new Intake();
        index = new StorageIndex();
        shooter = new Shooter();
 
      case SwerveTest:
        vision = new Vision(VisionConstants.APRIL_TAG_CAMERAS);


        drive = new Drivetrain(vision);
        driver = new GameControllerDriverConfig(drive, vision, arm, intake, index, shooter);
        operator = new Operator(intake, arm, index, shooter, drive, consumer);

        // Detected objects need access to the drivetrain
        //DetectedObject.setDrive(drive);
        
        //SignalLogger.start();

        driver.configureControls();
        operator.configureControls();
        initializeAutoBuilder();
        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
        registerCommands();
        PathGroupLoader.loadPathGroups();
 
        shuffleboardManager = new ShuffleBoardManager(drive, vision, shooter, arm, index, intake);
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
    AutoBuilder.configureHolonomic(
        () -> drive.getPose(),
        (pose) -> {
          drive.resetOdometry(pose);
        },
        () -> drive.getChassisSpeeds(),
        (chassisSpeeds) -> {
          drive.setChassisSpeeds(chassisSpeeds, false); // problem??
        },
        AutoConstants.config,
        getAllianceColorBooleanSupplier(),
        drive);
  }

  public void registerCommands() {

    // Stuff used in Choreo Paths
    NamedCommands.registerCommand("Intake", new IntakeNote(intake, index, arm, (ignored) -> {}).withTimeout(1));

    NamedCommands.registerCommand("Intake_Note_1.5_Sec", new IntakeNote(intake, index, arm, consumer).withTimeout(1.75));
    
     NamedCommands.registerCommand("Outtake_Note_1.50_Sec", new SequentialCommandGroup(
       new ParallelDeadlineGroup(
       new InstantCommand(() -> drive.setChassisSpeeds(new ChassisSpeeds(), true)),
       new WaitCommand(.75)),
     new WaitCommand(.75)
     ));

    NamedCommands.registerCommand("Intake_Note_2.5_Sec", new IntakeNote(intake, index, arm, consumer).withTimeout(2.5)); // 3 seconds used at SVR
    
    //Old
    NamedCommands.registerCommand("Outtake_Note_1.5_Sec", new SequentialCommandGroup(// TODO: This will end instantly
    // TODO: Don't use setChassisSpeeds(), use drive() instead and add the drivetrain as a parameter so it is a requirement
      new ParallelDeadlineGroup(new PrepareShooter(shooter, 1750),
      new WaitCommand(.75)),
      new WaitCommand(.75),
      new InstantCommand(()-> index.runIndex()),
      new WaitCommand(.75),
      new ParallelDeadlineGroup(new PrepareShooter(shooter, 0))
     ));

    // Whole time running
    NamedCommands.registerCommand("Set_Shooter",
      new SequentialCommandGroup(// TODO: This will end instantly
        new PrepareShooter(shooter, 1750),
        new WaitCommand(.75) ) );

    // NamedCommands.registerCommand("Set_Shooter",
    //   new SequentialCommandGroup(// TODO: This will end instantly
    //     new ParallelDeadlineGroup(new PrepareShooter(shooter, 1750),
    //         new WaitCommand(.75)),
    //     new WaitCommand(.75) ) );


    // Runs the Indexer
    NamedCommands.registerCommand("Outtake", new SequentialCommandGroup(
      new WaitCommand(.25),
      new InstantCommand(()-> index.runIndex()),
      new WaitCommand(.25)
    ));

    NamedCommands.registerCommand("Lower_Set_Shooter_Sabotage_Prep", new SequentialCommandGroup(
      new ParallelDeadlineGroup(new PrepareShooter(shooter, 50))
    ));

    NamedCommands.registerCommand("Sabotage_Second_Shot_Prep", new SequentialCommandGroup(
      new ParallelDeadlineGroup(new PrepareShooter(shooter, 1250))
    ));
    
      // new InstantCommand(() -> drive.setChassisSpeeds(new ChassisSpeeds(), true)),
      // new WaitCommand(.75)),
      // new InstantCommand(()-> index.runIndex()),
      // new WaitCommand(.5))); 
      //TODO: Stop index after command finishes

    NamedCommands.registerCommand("Prepare Shooter", new SequentialCommandGroup(new PrepareShooter(shooter, 1750), new WaitCommand(1)));
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



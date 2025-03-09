package frc.robot;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.auto_comm.FollowPathCommand;
import frc.robot.commands.drive_comm.DefaultDriveCommand;
import frc.robot.commands.gpm.IntakeCoral;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.commands.gpm.OuttakeCoralBasic;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.controls.PS5ControllerDriverConfig;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeAlpha;
import frc.robot.subsystems.outtake.OuttakeComp;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.Vision.DetectedObject;
import frc.robot.util.Vision.Vision;
import frc.robot.subsystems.climb.Climb;

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

    // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Controllers are defined here
  private BaseDriverConfig driver = null;
  @SuppressWarnings("unused")
  private Operator operator = null;

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
        outtake = new OuttakeComp();
        elevator = new Elevator();
        climb = new Climb();
        
      case BetaBot:
        indexer = new Indexer();
        intake = new Intake();
        //SmartDashboard.putData("commadn schedule", CommandScheduler.getInstance());
        vision = new Vision(VisionConstants.APRIL_TAG_CAMERAS);
        // fall-through
        //  SmartDashboard.putData("RunIntakeAndIndexer", new RunIntakeAndIndexer(intake, indexer));

      case Vivace:
      case Phil:
        if (robotId == RobotId.Phil) {
          outtake = new OuttakeAlpha();
        }
        if (outtake != null) {
          //SmartDashboard.putData("OuttakeCoralBasic", new OutakeMotors(intake, outtake));
          // SmartDashboard.putData("OuttakeCoralBasic", new OutakeMotors(intake, outtake));
          // SmartDashboard.putData("l4 outake", new ScoreL4(elevator, outtake));
        }
      case Vertigo:
        drive = new Drivetrain(vision, new GyroIOPigeon2());
        driver = new PS5ControllerDriverConfig(drive, elevator, intake, indexer, outtake, climb);
        //operator = new Operator(drive, elevator, intake, indexer, outtake, climb);

        // Detected objects need access to the drivetrain
        DetectedObject.setDrive(drive);
        
        //SignalLogger.start();

        driver.configureControls();
        //operator.configureControls();
        initializeAutoBuilder();
        registerCommands();
        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
        PathGroupLoader.loadPathGroups();      
        break;
      }

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);
    autoChooser = new LoggedDashboardChooser<>("auto selector");
   
    // TODO: verify this claim.
    // LiveWindow is causing periodic loop overruns
    LiveWindow.disableAllTelemetry();
    LiveWindow.setEnabled(false);
    

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
    if(intake != null && indexer != null && elevator != null){
      NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(intake, indexer, elevator, outtake));
    }
    if(elevator != null && outtake != null){

      NamedCommands.registerCommand("OuttakeCoral", new OuttakeCoral(outtake, elevator).withTimeout(1.5));
      NamedCommands.registerCommand("L4", new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT));

      NamedCommands.registerCommand("Lower Elevator", new InstantCommand(()->elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT)));
      
      NamedCommands.registerCommand("Score L4", new SequentialCommandGroup(
        new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT),
        new OuttakeCoral(outtake, elevator)
      ));

      
      NamedCommands.registerCommand("Score L3", new SequentialCommandGroup(
        new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT),
        new OuttakeCoral(outtake, elevator)
      ));

      NamedCommands.registerCommand("Score L2", new SequentialCommandGroup(
        new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT),
        new OuttakeCoral(outtake, elevator)
      ));
      
      NamedCommands.registerCommand("L3", new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT));
      NamedCommands.registerCommand("L2", new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT));
      NamedCommands.registerCommand("L1", new MoveElevator(elevator, ElevatorConstants.L1_SETPOINT));
    }
  }

  public void addPaths(){
        autoChooser.addDefaultOption("Do Nothing", new DoNothing());

        try {
            List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Right Side");
            
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
        autoChooser.addOption("Wait", new PathPlannerAuto("Wait Test"));

        autoChooser.addOption("Right Side", new PathPlannerAuto("Right Side"));
        autoChooser.addOption("Left Side", new PathPlannerAuto("Left Side"));
        autoChooser.addOption("Left Side Ground", new PathPlannerAuto("Left Side Ground"));

       
        // autoChooser.addOption("#1", new FollowPathCommand("#1", true, drive)
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator))
        // .andThen(new FollowPathCommand("#2", true, drive))
        // .andThen(new FollowPathCommand("#3", true, drive))
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator))
        // .andThen(new FollowPathCommand("#4", true, drive))
        // .andThen(new FollowPathCommand("#5", true, drive))
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator)));    

        
        if(elevator != null && outtake != null) {
         autoChooser.addOption("WaitTest", new FollowPathCommand("Tester", true, drive)
         .andThen(new OuttakeCoralBasic(outtake, ()->true))
         .andThen(new WaitCommand(3))
         .andThen(new FollowPathCommand("Next Tester", true, drive))
         );

          autoChooser.addOption("Center to G", new FollowPathCommand("Center to G", true, drive)
         .andThen(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT))
         .andThen(new OuttakeCoral(outtake, elevator)));

         autoChooser.addOption("Center to H", new FollowPathCommand("Center to H", true, drive)
         .andThen(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT))
         .andThen(new OuttakeCoral(outtake, elevator)));
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

  // 1.795 1.108
  public void interruptThreads(){
    odometryThread.interrupt();
    drivetrainThread.interrupt();
  }

  public boolean brownout() {
    if(RobotController.getBatteryVoltage() < 6.0) {
      return true;
    }
    else {
      return false;
    }
  }
}



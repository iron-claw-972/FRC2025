package frc.robot;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.commands.drive_comm.DefaultDriveCommand;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.commands.gpm.IntakeCoral;
import frc.robot.commands.gpm.MoveArm;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.commands.gpm.StationIntake;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.controls.PS5ControllerDriverConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeAlpha;
import frc.robot.subsystems.outtake.OuttakeComp;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.Vision.DetectedObject;
import frc.robot.util.Vision.Vision;

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
  private Arm arm = null;
  private Command auto = new DoNothing();

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;

  // Controllers are defined here
  private BaseDriverConfig driver = null;
  @SuppressWarnings("unused")
  private Operator operator = null;

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
        arm = new Arm();
        // Arm can move if the elevator is within tolerance of its safe setpoint or higher
        arm.setElevatorStowed(() -> elevator.getPosition() < ElevatorConstants.SAFE_SETPOINT - 0.025);
        // Elevator can only move down if the arm is in the intake setpoint
        elevator.setArmStowed(() -> arm.canMoveElevator());

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
        driver = new PS5ControllerDriverConfig(drive, elevator, intake, indexer, outtake, climb, arm);
        //operator = new Operator(drive, elevator, intake, indexer, outtake, climb);

        // Detected objects need access to the drivetrain
        DetectedObject.setDrive(drive);
        
        //SignalLogger.start();
        

        driver.configureControls();
        //operator.configureControls();
        
        initializeAutoBuilder();
        registerCommands();
        PathGroupLoader.loadPathGroups();
        // Load the auto command
        try {
          PathPlannerAuto.getPathGroupFromAutoFile("Left Side");
          auto = new PathPlannerAuto("Left Side");
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
        break;
      }

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    //addPaths(); 
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
          Logger.recordOutput("Auto/ChassisSpeeds", chassisSpeeds);
          drive.setChassisSpeeds(chassisSpeeds, false); // problem??
        },
        AutoConstants.AUTO_CONTROLLER,
        AutoConstants.CONFIG,
        getAllianceColorBooleanSupplier(),
        drive);
  }

  public void registerCommands() {
    if(intake != null && indexer != null && elevator != null && arm != null){
      NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(intake, indexer, elevator, outtake, arm));
      NamedCommands.registerCommand("lower intake", new InstantCommand(() -> intake.setAngle(IntakeConstants.INTAKE_SAFE_POINT)));
    }
    if(elevator != null && outtake != null && arm != null){
      NamedCommands.registerCommand("OuttakeCoral", new OuttakeCoral(outtake, elevator, arm).withTimeout(1.5));
      NamedCommands.registerCommand("L4", 
        new ParallelCommandGroup(
          new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT),
          new MoveArm(arm, ArmConstants.L4_SETPOINT_RIGHT)
        )
      );
      NamedCommands.registerCommand("backdrive", new InstantCommand(() -> outtake.setMotor(0.02)));

      NamedCommands.registerCommand("Lower Elevator", new SequentialCommandGroup(
        new InstantCommand(()->arm.setSetpoint(ArmConstants.INTAKE_SETPOINT)),
        new InstantCommand(()->elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT))
      ));
      
      NamedCommands.registerCommand("Score L4", new SequentialCommandGroup(
        new ParallelCommandGroup(
          new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT),
          new MoveArm(arm, ArmConstants.L4_SETPOINT_RIGHT)
        ),
        new OuttakeCoral(outtake, elevator, arm)
      ));

      NamedCommands.registerCommand("Score L3", new SequentialCommandGroup(
        new ParallelCommandGroup(
          new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT),
          new MoveArm(arm, ArmConstants.L2_L3_SETPOINT)
        ),
        new OuttakeCoral(outtake, elevator, arm)
      ));

      NamedCommands.registerCommand("Score L2", new SequentialCommandGroup(
        new ParallelCommandGroup(
          new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT),
          new MoveArm(arm, ArmConstants.L2_L3_SETPOINT)
        ),
        new OuttakeCoral(outtake, elevator, arm)
      ));
      
      NamedCommands.registerCommand("L3", 
        new ParallelCommandGroup(
          new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT),
          new MoveArm(arm, ArmConstants.L2_L3_SETPOINT)
        )
      );
      NamedCommands.registerCommand("L2", 
        new ParallelCommandGroup(
          new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT),
          new MoveArm(arm, ArmConstants.L2_L3_SETPOINT)
        )
      );
      NamedCommands.registerCommand("Station Setpoint", 
        new ParallelCommandGroup(
          new MoveElevator(elevator, ElevatorConstants.STATION_INTAKE_SETPOINT),
          new MoveArm(arm, ArmConstants.STATION_INTAKE_SETPOINT),
          new StationIntake(outtake)
        )
      );

      //NamedCommands.registerCommand("L1", new MoveElevator(elevator, ElevatorConstants.L1_SETPOINT));

      NamedCommands.registerCommand("Station Intake", new StationIntake(outtake));
    
      Pose2d blueStationRight = new Pose2d(1.722, 0.923, Rotation2d.fromDegrees(-36));
      Pose2d blueStationLeft = new Pose2d(blueStationRight.getX(), FieldConstants.FIELD_WIDTH-blueStationRight.getY(), Rotation2d.fromDegrees(-144));

      Pose2d blueStationIntakeLeft = new Pose2d(1.65, 7.4, Rotation2d.fromDegrees(-144-180));
      Pose2d blueStationIntakeRight = new Pose2d(1.526, 0.729, Rotation2d.fromDegrees(-144-180));
      
      Pose2d redStationRight = new Pose2d(FieldConstants.FIELD_LENGTH-blueStationRight.getX(), blueStationLeft.getY(), blueStationRight.getRotation().plus(new Rotation2d(Math.PI)));
      Pose2d redStationLeft = new Pose2d(FieldConstants.FIELD_LENGTH-blueStationLeft.getX(), blueStationRight.getY(), blueStationLeft.getRotation().plus(new Rotation2d(Math.PI)));
      NamedCommands.registerCommand("Drive To Left Station", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? redStationLeft : blueStationLeft));
      NamedCommands.registerCommand("Drive To Right Station Intake", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? redStationRight : blueStationIntakeRight));
      NamedCommands.registerCommand("Drive To Left Station Intake", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? redStationLeft : blueStationIntakeLeft));
      
      NamedCommands.registerCommand("Drive To Right Station", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? redStationRight : blueStationRight));
      NamedCommands.registerCommand("Drive To 6/19 Left", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? VisionConstants.REEF.RED_BRANCH_6_LEFT.l4Pose : VisionConstants.REEF.BLUE_BRANCH_19_LEFT.l4Pose).withTimeout(1));
      NamedCommands.registerCommand("Drive To 6/19 Right", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? VisionConstants.REEF.RED_BRANCH_6_RIGHT.l4Pose : VisionConstants.REEF.BLUE_BRANCH_19_RIGHT.l4Pose).withTimeout(1));
      NamedCommands.registerCommand("Drive To 7/18 Left", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? VisionConstants.REEF.RED_BRANCH_7_LEFT.l4Pose : VisionConstants.REEF.BLUE_BRANCH_18_LEFT.l4Pose));
      NamedCommands.registerCommand("Drive To 7/18 Right", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? VisionConstants.REEF.RED_BRANCH_7_RIGHT.l4Pose : VisionConstants.REEF.BLUE_BRANCH_18_RIGHT.l4Pose));
      NamedCommands.registerCommand("Drive To 10/21 Right", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? VisionConstants.REEF.RED_BRANCH_9_RIGHT.l4Pose : VisionConstants.REEF.BLUE_BRANCH_22_RIGHT.l4Pose));
      NamedCommands.registerCommand("Drive To 11/20 Left", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? VisionConstants.REEF.RED_BRANCH_11_LEFT.l4Pose : VisionConstants.REEF.BLUE_BRANCH_20_LEFT.l4Pose));
      NamedCommands.registerCommand("Drive To 11/20 Right", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? VisionConstants.REEF.RED_BRANCH_11_RIGHT.l4Pose : VisionConstants.REEF.BLUE_BRANCH_20_RIGHT.l4Pose));
      NamedCommands.registerCommand("Drive To 9/22 Left", new DriveToPose(drive, () -> DriverStation.getAlliance().get() 
      == DriverStation.Alliance.Red ? VisionConstants.REEF.RED_BRANCH_9_LEFT.l4Pose : VisionConstants.REEF.BLUE_BRANCH_22_LEFT.l4Pose)); 
      

      NamedCommands.registerCommand("Drive To 8/17 Left", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? VisionConstants.REEF.RED_BRANCH_8_LEFT.l4Pose : VisionConstants.REEF.BLUE_BRANCH_17_LEFT.l4Pose).withTimeout(1));
      NamedCommands.registerCommand("Drive To 8/17 Right", new DriveToPose(drive, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? VisionConstants.REEF.RED_BRANCH_8_RIGHT.l4Pose : VisionConstants.REEF.BLUE_BRANCH_17_RIGHT.l4Pose).withTimeout(1));
    }
  }

  // public void addPaths(){
  //       try {
  //           PathPlannerAuto.getPathGroupFromAutoFile("Left Side");
  //       } 
  //       catch (IOException | ParseException e) {
  //           e.printStackTrace();
  //       }
  //       autoChooser.addDefaultOption("Left Side", new PathPlannerAuto("Left Side"));

  //       autoChooser.addOption("Station Right Side", new PathPlannerAuto("Station Right Side"));
  //       autoChooser.addOption("Left Side Lollipop", new PathPlannerAuto("Left Side Lollipop"));
  //       autoChooser.addOption("Left Side Ground", new PathPlannerAuto("Left Side Ground"));

  //       if(intake != null && indexer != null && arm != null && elevator != null){
  //         autoChooser.addOption("One peice blue", 
  //         new SequentialCommandGroup(
  //           new InstantCommand(()->{
  //             drive.resetOdometry(new Pose2d(7.229,4.191, Rotation2d.fromDegrees(90.0)));
  //             intake.setAngle(IntakeConstants.INTAKE_SAFE_POINT);
  //           }),
  //           new DriveToPose(drive, () -> VisionConstants.REEF.BLUE_BRANCH_21_RIGHT.pose).withTimeout(8),
  //           new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT),
  //           new MoveArm(arm, ArmConstants.L4_SETPOINT),
  //           new OuttakeCoral(outtake, elevator, arm),
  //           new SequentialCommandGroup(new WaitCommand(0.1),
  //           new MoveArm(arm, ArmConstants.INTAKE_SETPOINT),
  //           new InstantCommand(()->elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT))),
  //           new InstantCommand(()-> intake.stow())
  //           ));
  //           autoChooser.addOption("One peice red", 
  //           new SequentialCommandGroup(
  //             new InstantCommand(()->{
  //               drive.resetOdometry(new Pose2d(FieldConstants.FIELD_LENGTH-7.229,FieldConstants.FIELD_WIDTH-4.191, Rotation2d.fromDegrees(-90.0)));
  //               intake.setAngle(IntakeConstants.INTAKE_SAFE_POINT);
  //             }),
  //             new DriveToPose(drive, () -> VisionConstants.REEF.RED_BRANCH_10_RIGHT.pose).withTimeout(8),
  //             new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT),
  //             new MoveArm(arm, ArmConstants.L4_SETPOINT),
  //             new OuttakeCoral(outtake, elevator, arm),
  //             new SequentialCommandGroup(new WaitCommand(0.1),
  //             new MoveArm(arm, ArmConstants.INTAKE_SETPOINT),
  //             new InstantCommand(()->elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT))),
  //             new InstantCommand(()-> intake.stow())
  //             ));
  //           }
  //         // autoChooser.addOption("#1", new FollowPathCommand("#1", true, drive)
  //       // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
  //       // .andThen(new OuttakeCoral(outtake, elevator, arm))
  //       // .andThen(new FollowPathCommand("#2", true, drive))
  //       // .andThen(new FollowPathCommand("#3", true, drive))
  //       // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
  //       // .andThen(new OuttakeCoral(outtake, elevator, arm))
  //       // .andThen(new FollowPathCommand("#4", true, drive))
  //       // .andThen(new FollowPathCommand("#5", true, drive))
  //       // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
  //       // .andThen(new OuttakeCoral(outtake, elevator, arm)));    

        
  //       if(elevator != null && outtake != null) {
  //        autoChooser.addOption("WaitTest", new FollowPathCommand("Tester", true, drive)
  //        .andThen(new OuttakeCoralBasic(outtake, ()->true, ()->false))
  //        .andThen(new WaitCommand(3))
  //        .andThen(new FollowPathCommand("Next Tester", true, drive))
  //        );

  //         autoChooser.addOption("Center to G", new FollowPathCommand("Center to G", true, drive)
  //        .andThen(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT))
  //        .andThen(new OuttakeCoral(outtake, elevator, arm)));

  //        autoChooser.addOption("Center to H", new FollowPathCommand("Center to H", true, drive)
  //        .andThen(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT))
  //        .andThen(new OuttakeCoral(outtake, elevator, arm)));
  //       }
  // }

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

  public boolean brownout() {
    if(RobotController.getBatteryVoltage() < 6.0) {
      return true;
    }
    else {
      return false;
    }
  }

  public Command getAutoCommand(){
    return auto;
  }

  // Logged psoitins of subsystems
  public double elevatorHeightLogged(){
    return elevator == null ? 0 : elevator.getPosition();
  }
  public double armAngleLogged(){
    return arm == null ? 0 : arm.getAngle() + 90;
  }
  public double intakeAngleLogged(){
    return intake == null ? -90 : -intake.getPivotAngle();
  }
  public double climbAngleLogged(){
    return climb == null ? 0 : climb.getEstimatedClimbAngle();
  }

  public void logComponents(){
    if(!Constants.LOG_MECHANISMS) return;
    
    Logger.recordOutput(
      "ComponentPoses", 
      new Pose3d[] {
        //intake
        new Pose3d(0,-0.25,0.27, new Rotation3d(Units.degreesToRadians(intakeAngleLogged()), 0.0, 0.0)),
        //climb
        new Pose3d(0,0,0, new Rotation3d(0.0,climbAngleLogged(), 0.0)),
        //arm
        new Pose3d(0,0.110, 0.388 + elevatorHeightLogged(), new Rotation3d(Units.degreesToRadians(armAngleLogged()), 0.0, 0.0)),
        //elevator 1
        new Pose3d(0,0,0, new Rotation3d(0.0, 0.0, 0.0)),
        //elevator 2
        new Pose3d(0,0, elevatorHeightLogged()/3, new Rotation3d(0.0, 0.0, 0.0)),
        //elevator 3
        new Pose3d(0,0, elevatorHeightLogged()*2/3, new Rotation3d(0.0, 0.0, 0.0)),
        //elevator 4
        new Pose3d(0,0, elevatorHeightLogged(), new Rotation3d(0.0, 0.0, 0.0)),
        //indexer
        new Pose3d(0,0,0, new Rotation3d(0.0, 0.0, 0.0)),
      }
    );
  }
}



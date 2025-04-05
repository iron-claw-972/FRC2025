package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.commands.gpm.IntakeAlgae;
import frc.robot.commands.gpm.IntakeAlgaeArm;
import frc.robot.commands.gpm.IntakeCoral;
import frc.robot.commands.gpm.MoveArm;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.NetSetpoint;
import frc.robot.commands.gpm.OuttakeAlgae;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.commands.gpm.ResetClimb;
import frc.robot.commands.gpm.ReverseMotors;
import frc.robot.commands.gpm.StationIntake;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  public static final boolean singleAlignmentButton = true;

  private final GameController driver = new GameController(Constants.DRIVER_JOY);
  private final BooleanSupplier slowModeSupplier = driver.get(Button.RIGHT_JOY);
  private final Elevator elevator;
  private final Intake intake;
  private final Indexer indexer;
  private final Outtake outtake;
  private final Climb climb;
  private final Arm arm;
  private int alignmentDirection = 0;
  private Pose2d alignmentPose = null;

  public GameControllerDriverConfig(Drivetrain drive, Elevator elevator, Intake intake, Indexer indexer,
      Outtake outtake, Climb climb, Arm arm) {
    super(drive);
    this.elevator = elevator;
    this.intake = intake;
    this.indexer = indexer;
    this.outtake = outtake;
    this.climb = climb;
    this.arm = arm;
  }

  @Override
  public void configureControls() {
    Trigger menu = driver.get(Button.LEFT_JOY);

    // Elevator setpoints
    if (elevator != null && arm != null) {
      driver.get(Button.BACK).and(menu.negate()).onTrue(
          new SequentialCommandGroup(
              new MoveElevator(elevator, ElevatorConstants.L1_SETPOINT),
              new MoveArm(arm, ArmConstants.L1_SETPOINT)));

      driver.get(driver.LEFT_TRIGGER_BUTTON).onTrue(
          new SequentialCommandGroup(
              new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT),
              new MoveArm(arm, ArmConstants.L4_SETPOINT_RIGHT)));

      Command l2Coral = new SequentialCommandGroup(
          new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT),
          new MoveArm(arm, ArmConstants.L2_L3_SETPOINT));
      Command l3Coral = new SequentialCommandGroup(
          new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT),
          new MoveArm(arm, ArmConstants.L2_L3_SETPOINT));
      Command l2Algae = new ParallelCommandGroup(
          new MoveElevator(elevator, ElevatorConstants.BOTTOM_ALGAE_SETPOINT),
          new MoveArm(arm, ArmConstants.ALGAE_SETPOINT)).andThen(new IntakeAlgaeArm(outtake));
      Command l3Algae = new ParallelCommandGroup(
          new MoveElevator(elevator, ElevatorConstants.TOP_ALGAE_SETPOINT),
          new MoveArm(arm, ArmConstants.ALGAE_SETPOINT)).andThen(new IntakeAlgaeArm(outtake));
      driver.get(Button.RB).whileTrue(new ConditionalCommand(l2Algae, new InstantCommand(l2Coral::schedule), menu));
      driver.get(Button.LB).whileTrue(new ConditionalCommand(l3Algae, new InstantCommand(l3Coral::schedule), menu));

      // Processor setpoint
      driver.get(Button.Y).and(menu.negate()).onTrue(
          new ParallelCommandGroup(
              new MoveElevator(elevator, ElevatorConstants.SAFE_SETPOINT + 0.001),
              new MoveArm(arm, ArmConstants.PROCESSOR_SETPOINT)));
      driver.get(DPad.UP).onTrue(new NetSetpoint(elevator, arm, getDrivetrain()));
    }
    // Intake/outtake
    Trigger r3 = driver.get(Button.RIGHT_JOY);
    
    if (intake != null && indexer != null && elevator != null && outtake != null && arm != null) {// && elevator != null){
      boolean toggle = true;
      Command intakeCoral = new IntakeCoral(intake, indexer, elevator, outtake, arm);
      Command intakeAlgae = new IntakeAlgae(intake);
      driver.get(Button.A).onTrue(new InstantCommand(() -> {
        if (r3.getAsBoolean())
          return;
        if (menu.getAsBoolean()) {
          intakeAlgae.schedule();
        } else {
          if (toggle) {
            if (intakeCoral.isScheduled()) {
              intakeCoral.cancel();
            } else {
              intakeCoral.schedule();
            }
          } else {
            intakeCoral.schedule();
          }
        }
      })).onFalse(new InstantCommand(() -> {
        if (!toggle) {
          intakeCoral.cancel();
        }
        intakeAlgae.cancel();
      }));
      // On true, run the command to start intaking
      // On false, run the command to finish intaking if it has a coral
      Command startIntake = new StationIntake(outtake);
      // Command finishIntake = new FinishStationIntake(intake, indexer, elevator,
      // outtake);
      driver.get(Button.A).and(r3).and(menu.negate()).onTrue(startIntake)
          .onFalse(new InstantCommand(() -> {
            if (!startIntake.isScheduled()) {
              // finishIntake.schedule();
            } else {
              startIntake.cancel();
            }
          }));
    }

    if (intake != null && outtake != null && arm != null && elevator != null) {
      driver.get(DPad.DOWN).and(menu).onTrue(new SequentialCommandGroup(
          new OuttakeAlgae(outtake, intake),
          new InstantCommand(() -> {
            arm.setSetpoint(ArmConstants.START_ANGLE);
            elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
            getDrivetrain().setIsAlign(false);
          }, elevator)));
    }

    if (outtake != null && elevator != null && arm != null) {
      driver.get(DPad.DOWN).and(menu.negate())
          .onTrue(new OuttakeCoral(outtake, elevator, arm)
              .alongWith(new InstantCommand(() -> getDrivetrain().setDesiredPose(() -> null)))
              .andThen(new SequentialCommandGroup(new MoveArm(arm, ArmConstants.INTAKE_SETPOINT),
                  new MoveElevator(elevator, ElevatorConstants.STOW_SETPOINT))));
    }
    driver.get(DPad.DOWN).and(menu.negate()).onTrue(new InstantCommand(() -> {
    }, getDrivetrain()));
    if (intake != null && indexer != null && outtake != null) {
      driver.get(Button.B).and(menu.negate()).whileTrue(new ReverseMotors(intake, indexer, outtake));
    }

    // Climb
    if (climb != null) {
      driver.get(Button.X).and(menu.negate())
          .toggleOnTrue(new StartEndCommand(() -> climb.extend(), () -> climb.climb(), climb));
      if (intake != null) {
        driver.get(Button.X).and(menu.negate()).onTrue(new InstantCommand(() -> intake.setAngle(65), intake));
      }
      driver.get(Button.X).and(menu).whileTrue(new ResetClimb(climb));
      driver.get(driver.RIGHT_TRIGGER_BUTTON).and(menu).onTrue(new InstantCommand(() -> climb.stow(), climb));
    }

    // Alignment
    if (singleAlignmentButton) {
      driver.get(DPad.LEFT).toggleOnTrue(new InstantCommand(() -> {
        setAlignmentDirection();
        setAlignmentPose(false, true);
      }).andThen(new DriveToPose(getDrivetrain(), () -> alignmentPose)));
      driver.get(DPad.RIGHT).toggleOnTrue(new InstantCommand(() -> {
        setAlignmentDirection();
        setAlignmentPose(false, false);
      }).andThen(new DriveToPose(getDrivetrain(), () -> alignmentPose)));
    } else {
      driver.get(DPad.LEFT).onTrue(new InstantCommand(() -> setAlignmentPose(false, true))
          .andThen(new DriveToPose(getDrivetrain(), () -> alignmentPose)));
      driver.get(DPad.RIGHT).onTrue(new InstantCommand(() -> setAlignmentPose(false, false))
          .andThen(new DriveToPose(getDrivetrain(), () -> alignmentPose)));
    }

    // Reset yaw to be away from driver
    driver.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
        new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI))));

    // Cancel commands
    driver.get(driver.RIGHT_TRIGGER_BUTTON).and(menu.negate()).onTrue(new InstantCommand(() -> {
      if (elevator != null) {
        if (outtake != null && outtake.coralLoaded()) {
          elevator.setSetpoint(ElevatorConstants.INTAKE_STOW_SETPOINT);
        } else {
          elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
        }
      }
      if (outtake != null) {
        outtake.stop();
      }
      if (intake != null) {
        intake.stow();
        intake.deactivate();
      }
      if (indexer != null) {
        indexer.stop();
      }
      if (climb != null) {
        climb.stow();
      }
      if (arm != null) {
        if (outtake != null && outtake.coralLoaded()) {
          arm.setSetpoint(ArmConstants.STOW_SETPOINT);
        } else {
          arm.setSetpoint(ArmConstants.START_ANGLE);
        }
      }
      getDrivetrain().setIsAlign(false);
      getDrivetrain().setDesiredPose(() -> null);
      CommandScheduler.getInstance().cancelAll();
    }));
  }

  private void setAlignmentDirection() {
    Translation2d drivePose = getDrivetrain().getPose().getTranslation();
    int closestDirection = 0;
    double closestDist = 20;
    boolean isRed = Robot.getAlliance() == Alliance.Red;
    int start = isRed ? 5 : 16;
    for (int i = 0; i < 6; i++) {
      double dist = FieldConstants.APRIL_TAGS.get(start + i).pose.toPose2d().getTranslation().getDistance(drivePose);
      if (dist < closestDist) {
        closestDist = dist;
        closestDirection = i;
      }
    }
    if (isRed) {
      closestDirection = (8 - closestDirection) % 6;
    }
    alignmentDirection = closestDirection;
  }

  /**
   * Sets the drivetrain's alignment pose to the selected position
   * 
   * @param isAlgae True for algae, false for branches
   * @param isLeft  True for left branch, false for right, ignored for algae
   */
  private void setAlignmentPose(boolean isAlgae, boolean isLeft) {
    int id = Robot.getAlliance() == Alliance.Blue ? alignmentDirection + 17
        : (8 - alignmentDirection) % 6 + 6;
    if (isAlgae) {
      alignmentPose = VisionConstants.REEF.fromAprilTagIdAlgae(id).pose;
    } else {
      alignmentPose = VisionConstants.REEF.fromAprilTagIdAndPose(id, isLeft).pose;
    }
  }

  @Override
  public double getRawForwardTranslation() {
    return driver.get(Axis.LEFT_Y);
  }

  @Override
  public double getRawSideTranslation() {
    return driver.get(Axis.LEFT_X);
  }

  @Override
  public double getRawRotation() {
    return driver.get(Axis.RIGHT_X);
  }

  @Override
  public double getRawHeadingAngle() {
    return Math.atan2(driver.get(Axis.RIGHT_X), -driver.get(Axis.RIGHT_Y)) - Math.PI / 2;
  }

  @Override
  public double getRawHeadingMagnitude() {
    return Math.hypot(driver.get(Axis.RIGHT_X), driver.get(Axis.RIGHT_Y));
  }

  @Override
  public boolean getIsSlowMode() {
    return slowModeSupplier.getAsBoolean();
  }

  @Override
  public boolean getIsAlign() {
    return false;
    // return kDriver.LEFT_TRIGGER_BUTTON.getAsBoolean();
  }

  public GameController getGameController() {
    return driver;
  }

}

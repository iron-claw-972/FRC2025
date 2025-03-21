package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.commands.gpm.IntakeAlgaeArm;
import frc.robot.commands.gpm.MoveArm;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.NetSetpoint;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.Vision.Vision;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  private final GameController kDriver = new GameController(Constants.DRIVER_JOY);

  private final Vision vision;
      private final Elevator elevator;
    private final Intake intake;
    private final Indexer indexer;
    private final Outtake outtake;
    private final Climb climb;
    private final Arm arm;

  public GameControllerDriverConfig(Drivetrain drive, Vision vision, Elevator elevator, Intake intake, Indexer indexer, Outtake outtake, Climb climb, Arm arm) {
    super(drive);
    this.vision = vision;
    this.elevator = elevator;
    this.intake = intake;
    this.indexer = indexer;
    this.outtake = outtake;
    this.climb = climb;
    this.arm = arm;
  }

  @SuppressWarnings("unused")
  @Override
  public void configureControls() {
    // TODO: Update to match PS5ControllerDriverConfig

    Trigger menu = kDriver.get(Button.LEFT_JOY);

    // Reset yaw to be away from driver
    //kDriver.setRumble(RumbleStatus.RUMBLE_ON);
    kDriver.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
        new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI))));

// Elevator setpoints
        if(elevator != null && outtake != null && arm != null) {
            kDriver.get(Button.BACK).and(menu.negate()).onTrue(
                new SequentialCommandGroup(
                    new MoveElevator(elevator, ElevatorConstants.L1_SETPOINT),
                    new MoveArm(arm, ArmConstants.L1_SETPOINT)
                )
            );

            kDriver.get(kDriver.LEFT_TRIGGER_BUTTON).onTrue(
                new SequentialCommandGroup(
                    new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT),
                    new MoveArm(arm, ArmConstants.L4_SETPOINT)
                )
            );

            Command l2Coral = new SequentialCommandGroup(
                new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT),
                new MoveArm(arm, ArmConstants.L2_L3_SETPOINT)
            );
            Command l3Coral = new SequentialCommandGroup(
                new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT),
                new MoveArm(arm, ArmConstants.L2_L3_SETPOINT)
            );
            Command l2Algae = new ParallelCommandGroup(
                new MoveElevator(elevator, ElevatorConstants.BOTTOM_ALGAE_SETPOINT),
                new MoveArm(arm, ArmConstants.ALGAE_SETPOINT)).andThen(new IntakeAlgaeArm(outtake));
            Command l3Algae = new ParallelCommandGroup(
                new MoveElevator(elevator, ElevatorConstants.TOP_ALGAE_SETPOINT),
                new MoveArm(arm, ArmConstants.ALGAE_SETPOINT)).andThen(new IntakeAlgaeArm(outtake));
            kDriver.get(Button.RB).whileTrue(new ConditionalCommand(l2Algae, new InstantCommand(l2Coral::schedule), menu));
            kDriver.get(Button.LB).whileTrue(new ConditionalCommand(l3Algae, new InstantCommand(l3Coral::schedule), menu));
    
            //Processor setpoint
            kDriver.get(Button.Y).and(menu.negate()).onTrue(
                new ParallelCommandGroup(
                    new MoveElevator(elevator, ElevatorConstants.SAFE_SETPOINT + 0.001),
                    new MoveArm(arm, ArmConstants.PROCESSOR_SETPOINT)
                )
            );
            kDriver.get(DPad.UP).onTrue(new NetSetpoint(elevator, arm, getDrivetrain()));
        }


    // set the wheels to X
    kDriver.get(Button.X).whileTrue(new SetFormationX(super.getDrivetrain()));
    // Enable state deadband after setting formation to X
    kDriver.get(Button.X).onFalse(new InstantCommand(()->getDrivetrain().setStateDeadband(true)));

    // Resets the modules to absolute if they are having the unresolved zeroing
    // error
    kDriver.get(Button.RB).onTrue(new InstantCommand(() -> getDrivetrain().resetModulesToAbsolute()));

    kDriver.get(Button.BACK).onTrue(new InstantCommand(()->getDrivetrain().getSwerveModulePose().reset()));

    if(vision != null && VisionConstants.DRIVER_ASSIST_MODE > 0){
      // This will only be true when it is equal to 1, but <=1 avoids a warning for comparing identical expressions
      if(VisionConstants.DRIVER_ASSIST_MODE <= 1){
        // Currently does nothing
      }else{
        (new Trigger(kDriver.LEFT_TRIGGER_BUTTON))
          .onTrue(new InstantCommand(()->getDrivetrain().setDesiredPose(()->vision.getBestGamePiece(Units.degreesToRadians(60), false).pose.toPose2d())))
          .onFalse(new InstantCommand(()->getDrivetrain().setDesiredPose(()->null)));
      }
    }
  }

  @Override
  public double getRawForwardTranslation() {
    return kDriver.get(Axis.LEFT_Y);
  }

  @Override
  public double getRawSideTranslation() {
    return kDriver.get(Axis.LEFT_X);
  }

  @Override
  public double getRawRotation() {
    return kDriver.get(Axis.RIGHT_X);
  }

  @Override
  public double getRawHeadingAngle() {
    return Math.atan2(kDriver.get(Axis.RIGHT_X), -kDriver.get(Axis.RIGHT_Y)) - Math.PI / 2;
  }

  @Override
  public double getRawHeadingMagnitude() {
    return Math.hypot(kDriver.get(Axis.RIGHT_X), kDriver.get(Axis.RIGHT_Y));
  }

  @Override
  public boolean getIsSlowMode() {
    return kDriver.RIGHT_TRIGGER_BUTTON.getAsBoolean();
  }

  @Override
  public boolean getIsAlign() {
    return false;
    // return kDriver.LEFT_TRIGGER_BUTTON.getAsBoolean();
  }

  public GameController getGameController(){
    return kDriver;
  }

}

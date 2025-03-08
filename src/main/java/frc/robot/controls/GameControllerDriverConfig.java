package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.commands.vision.DriverAssistIntake;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.Vision;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  private final GameController kDriver = new GameController(Constants.DRIVER_JOY);

  private final Vision vision;

  public GameControllerDriverConfig(Drivetrain drive, Vision vision) {
    super(drive);
    this.vision = vision;
  }

  @SuppressWarnings("unused")
  @Override
  public void configureControls() {
    // Reset yaw to be away from driver
    //kDriver.setRumble(RumbleStatus.RUMBLE_ON);
    kDriver.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
        new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI))));

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
        (new Trigger(kDriver.LEFT_TRIGGER_BUTTON)).whileTrue(new DriverAssistIntake(getDrivetrain(), this, vision));
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

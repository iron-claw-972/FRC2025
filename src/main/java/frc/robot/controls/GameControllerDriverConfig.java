package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.MathUtils;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  private final GameController kDriver = new GameController(Constants.DRIVER_JOY);

  public GameControllerDriverConfig(Drivetrain drive) {
    super(drive);
  }

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
    return MathUtils.calculateHypotenuse(kDriver.get(Axis.RIGHT_X), kDriver.get(Axis.RIGHT_Y));
  }

  @Override
  public boolean getIsSlowMode() {
    return kDriver.RIGHT_TRIGGER_BUTTON.getAsBoolean();
  }

  @Override
  public boolean getIsAlign() {
    return kDriver.LEFT_TRIGGER_BUTTON.getAsBoolean();
  }

  public GameController getGameController(){
    return kDriver;
  }

}

package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.GoToPose;
import frc.robot.commands.OuttakeAmp;
import frc.robot.commands.SysIDDriveCommand;
import frc.robot.commands.drive_comm.SetFormationX;
//import frc.robot.commands.vision.AcquireGamePiece;
import frc.robot.constants.Constants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.util.MathUtils;
import frc.robot.util.Vision;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  private final GameController kDriver = new GameController(Constants.DRIVER_JOY);
  private Vision vision;
  private Arm arm;
  private Intake intake;
  private StorageIndex index;
  private Shooter shooter;

  public GameControllerDriverConfig(Drivetrain drive, Vision vision, Arm arm, Intake intake, StorageIndex index, Shooter shooter) {
    super(drive);
    this.vision = vision;
    this.arm = arm;
    this.index = index;
    this.intake = intake;
    this.shooter = shooter;
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

    // if(VisionConstants.OBJECT_DETECTION_ENABLED){
    //   if(intake != null && index != null && arm != null){
    //     kDriver.get(Button.RIGHT_JOY).whileTrue(new AcquireGamePiece(()->vision.getBestGamePiece(Math.PI/2), getDrivetrain(), intake, index, arm));
    //   }
    // }

    // Resets the modules to absolute if they are having the unresolved zeroing
    // error
    kDriver.get(Button.RB).onTrue(new InstantCommand(() -> getDrivetrain().resetModulesToAbsolute()));
    //kDriver.get(Button.RB).onTrue(new SysIDDriveCommand(getDrivetrain()));
    kDriver.get(Button.X).whileTrue(new GoToPose(()->
      Robot.getAlliance() == Alliance.Red ? VisionConstants.RED_SUBWOOFER_LEFT
      : VisionConstants.BLUE_SUBWOOFER_LEFT,
      getDrivetrain()));
    kDriver.get(Button.Y).whileTrue(new GoToPose(()->
      Robot.getAlliance() == Alliance.Red ? VisionConstants.RED_SUBWOOFER_CENTER
      : VisionConstants.BLUE_SUBWOOFER_CENTER,
       getDrivetrain()));
    kDriver.get(Button.B).whileTrue(new GoToPose(()->
      Robot.getAlliance() == Alliance.Red ? VisionConstants.RED_SUBWOOFER_RIGHT
      : VisionConstants.BLUE_SUBWOOFER_RIGHT,
      getDrivetrain()));


    // Amp alignment
    if(arm != null && index != null && shooter != null){
      // kDriver.get(Button.B).whileTrue(new OuttakeAmp(arm, index, shooter, getDrivetrain()));
      kDriver.get(Button.A).whileTrue(new OuttakeAmp(getDrivetrain()));
    }else{
      kDriver.get(Button.A).whileTrue(new OuttakeAmp(getDrivetrain()));
    }
    // Podium alignment
    kDriver.get(Button.LB)
        .whileTrue(new GoToPose(
            () -> Robot.getAlliance() == Alliance.Blue ? VisionConstants.BLUE_PODIUM_POSE
                : VisionConstants.RED_PODIUM_POSE,
            getDrivetrain()));
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

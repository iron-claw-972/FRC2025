package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.commands.gpm.IndexerFeed;
import frc.robot.commands.gpm.IntakeNote;
import frc.robot.commands.gpm.PrepareShooter;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.subsystems.gpm.Intake.Mode;
import frc.robot.util.MathUtils;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  private final GameController kDriver = new GameController(Constants.DRIVER_JOY);
  private Arm arm;
  private StorageIndex index;
  private Shooter shooter;
  private Intake intake;

  public GameControllerDriverConfig(Drivetrain drive, Arm arm, Intake intake, StorageIndex index, Shooter shooter) {
    super(drive);
    this.arm = arm;
    this.intake = intake;
    this.index = index;
    this.shooter = shooter;
  }

  @Override
  public void configureControls() {
    // Reset yaw to be away from driver
    kDriver.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
        new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI))));

    // set the wheels to X
    kDriver.get(Button.X).whileTrue(new SetFormationX(getDrivetrain()));
    // Enable state deadband after setting formation to X
    kDriver.get(Button.X).onFalse(new InstantCommand(()->getDrivetrain().setStateDeadband(true)));

    // Resets the modules to absolute if they are having the unresolved zeroing
    // error
    kDriver.get(Button.RB).onTrue(new InstantCommand(() -> getDrivetrain().resetModulesToAbsolute()));

    kDriver.get(Button.A).whileTrue(new IntakeNote(intake, index, arm, rumble->{}));
    kDriver.get(Button.LB).onTrue(new PrepareShooter(shooter, Shooter.addSlip(Shooter.shooterSpeedToRPM(ShooterConstants.SHOOT_SPEED_MPS))));
    new Trigger(kDriver.LEFT_TRIGGER_BUTTON).onTrue(new SequentialCommandGroup(
      new ConditionalCommand(new SequentialCommandGroup(
        new PrepareShooter(shooter, Shooter.addSlip(Shooter.shooterSpeedToRPM(ShooterConstants.SHOOT_SPEED_MPS))),
        new WaitCommand(2)
      ), new DoNothing(), ()->shooter.getLeftMotorRPM()<750),
      new IndexerFeed(index),
      new PrepareShooter(shooter, 0)
    ));
    kDriver.get(Button.B).onTrue(new InstantCommand(() -> index.ejectBack(), index))
      .onFalse(new InstantCommand(() -> index.stopIndex(), index))
      .onTrue(new InstantCommand(() -> intake.setMode(Mode.ReverseMotors), intake))
      .onFalse(new InstantCommand(() -> intake.setMode(Mode.DISABLED), intake));
    kDriver.get(Button.Y).whileTrue(new InstantCommand(()->{
      intake.setMode(Mode.INTAKE);
      index.runIndex();
      shooter.setTargetVelocity(ShooterConstants.SHOOT_SPEED_MPS);
    }, intake, index, shooter, arm).andThen(new RunCommand(()->{}))); // Never ends until you let go
    kDriver.get(Button.Y).onFalse(new InstantCommand(()->{
      intake.setMode(Mode.DISABLED);
      index.stopIndex();
      shooter.setTargetVelocity(0);
    }));
    kDriver.get(Button.BACK).onTrue(new InstantCommand(()->{
      intake.setMode(Mode.DISABLED);
      index.stopIndex();
      shooter.setTargetRPM(0);
      CommandScheduler.getInstance().cancelAll();
    }));
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
    return true;
  }

  @Override
  public boolean getIsAlign() {
    return false;
  }

  public GameController getGameController(){
    return kDriver;
  }

}

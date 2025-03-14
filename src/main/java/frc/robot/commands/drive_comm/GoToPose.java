package frc.robot.commands.drive_comm;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.commands.SupplierCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
* Moves the robot to a pose using PathPlanner
*/
public class GoToPose extends SequentialCommandGroup {

  private Drivetrain drive;
  private Supplier<Pose2d> poseSupplier;
  private double maxSpeed;
  private double maxAccel;

  /**
   * Uses PathPlanner to go to a pose
   * @param poseSupplier The supplier for the pose to use
   * @param drive The drivetrain
   */
  public GoToPose(Supplier<Pose2d> poseSupplier, Drivetrain drive) {
    this(poseSupplier, AutoConstants.MAX_AUTO_SPEED, AutoConstants.MAX_AUTO_ACCEL, drive);
  }
  public GoToPose(Pose2d pose, Drivetrain drive){
    this(()->pose, drive);
  }

  /**
   * Uses PathPlanner to go to a pose
   * @param poseSupplier The supplier for the pose to use
   * @param maxSpeed The maximum speed to use
   * @param maxAccel The maximum acceleration to use
   * @param drive The drivetrain
   */
  public GoToPose(Supplier<Pose2d> poseSupplier, double maxSpeed, double maxAccel, Drivetrain drive) {
    this.poseSupplier = poseSupplier;
    this.maxSpeed = maxSpeed;
    this.maxAccel = maxAccel;
    this.drive = drive;
    addCommands(
      new InstantCommand(()->drive.setVisionEnabled(VisionConstants.ENABLED_GO_TO_POSE)),
      new SupplierCommand(() -> createCommand(), drive).handleInterrupt(()->drive.setVisionEnabled(true)),
      new InstantCommand(()->drive.setVisionEnabled(true))
    );
  }

  /**
   * Creates the PathPlanner command and schedules it
   */
  public Command createCommand() {
    Pose2d pose = poseSupplier.get();
    if(pose==null){
      return new DoNothing();
    }
    Command command = AutoBuilder.pathfindToPose(
      pose,
      new PathConstraints(maxSpeed, maxAccel, DriveConstants.MAX_ANGULAR_SPEED, DriveConstants.MAX_ANGULAR_ACCEL),
      0
    );

    // get the distance to the pose.
    double dist = drive.getPose().minus(pose).getTranslation().getNorm();

    // if greater than 3m or less than 2 cm, don't run it. If the path is too small pathplanner makes weird paths.
    if (dist > 3) {
      command = new DoNothing();
      DriverStation.reportWarning("Alignment Path too long, doing nothing, GoToPose.java", false);
    } else if (dist < 0.02) {
      command = new DoNothing();
      DriverStation.reportWarning("Alignment Path too short, doing nothing, GoToPose.java", false);
    }

    return command;
  }
}

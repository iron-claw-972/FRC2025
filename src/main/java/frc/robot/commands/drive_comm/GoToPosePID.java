package frc.robot.commands.drive_comm;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Runs the chassis PIDs to move the robot to a specific pose. 
 */
public class GoToPosePID extends Command {

  private Drivetrain drive; 
  
  private Supplier<Pose2d> poseSupplier;
  private Pose2d pose;
  
  /**
   * Runs the chassis PIDs to move the robot to a specific pose. 
   * @param pose The pose supplier to go to
   * @param drive The drivetrain
   */
  public GoToPosePID(Supplier<Pose2d> pose, Drivetrain drive) {
    this.drive = drive;
    this.poseSupplier = pose;

    addRequirements(drive);
  }

  public GoToPosePID(Pose2d pose, Drivetrain drive){
    this(()->pose, drive);
  }

  @Override
  public void initialize() {
    pose = poseSupplier.get();
    drive.setVisionEnabled(VisionConstants.ENABLED_GO_TO_POSE);
  }

  @Override
  public void execute() {
    if(pose == null) {
      return;
    }

    drive.driveWithPID(pose.getX(), pose.getY(), pose.getRotation().getRadians()); 
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    drive.setVisionEnabled(true);
  }

  @Override
    public boolean isFinished() {
        return pose == null || drive.getXController().atSetpoint() && drive.getYController().atSetpoint() && drive.getRotationController().atSetpoint();
    }
}
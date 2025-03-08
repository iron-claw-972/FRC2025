package frc.robot.commands.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Aims the robot at the closest April tag
 */
public class AimAtTag extends Command {
  Drivetrain m_drive;
  PIDController m_pid;

  /**
   * Aims the robot at the closest April tag
   * @param drive The drivetrain
   */
  public AimAtTag(Drivetrain drive){
    m_drive = drive;
    // Copy drive PID and changetolerance
    m_pid = new PIDController(
      m_drive.getRotationController().getP(),
      m_drive.getRotationController().getI(),
      m_drive.getRotationController().getD()
    );
    m_pid.setTolerance(Units.degreesToRadians(1));
    addRequirements(drive);
  }

  /**
   * Gets the closest tag and sets the setpoint to aim at it
   */
  @Override
  public void initialize(){
    double dist = Double.POSITIVE_INFINITY;
    Translation2d closest = new Translation2d();
    Translation2d driveTranslation = m_drive.getPose().getTranslation();
    for(AprilTag tag : FieldConstants.APRIL_TAGS){
      Translation2d translation = tag.pose.toPose2d().getTranslation();
      double dist2 = driveTranslation.getDistance(translation);
      if(dist2 < dist){
        dist = dist2;
        closest = translation;
      }
    }
    m_pid.reset();
    m_pid.setSetpoint(Math.atan2(closest.getY() - driveTranslation.getY(), closest.getX() - driveTranslation.getX()));
  }
  
  /**
   * Runs the PID
   */
  @Override
  public void execute() {
    double angle = m_drive.getPose().getRotation().getRadians();
    // If the distance between the angles is more than 180 degrees, use an identical angle ±360 degrees
    if(angle - m_pid.getSetpoint() > Math.PI){
      angle -= 2*Math.PI;
    }else if(angle - m_pid.getSetpoint() < -Math.PI){
      angle += 2*Math.PI;
    }
    double speed = m_pid.calculate(angle);
    m_drive.drive(0, 0, speed, true, false);
  }

  /**
   * Stops the drivetrain
   * @param interrupted If the command is interrupted
   */
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  /**
   * Returns if the command is finished
   * @return If the PID is at the setpoint
   */
  @Override
  public boolean isFinished() {
    return m_pid.atSetpoint();
  }
}


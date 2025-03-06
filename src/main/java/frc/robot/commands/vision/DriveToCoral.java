package frc.robot.commands.vision;

import java.util.function.Supplier;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DetectedObject;

/**
 * Moves toward the detected object
 * <p>Only works with the front camera
 */
public class DriveToCoral extends DriveToPose {
  private static final boolean constantUpdate = false;

  private static Pose2d tempPose;

  private Supplier<DetectedObject> objectSupplier;

  /**
   * Moves toward the detected object
   * @param detectedObject The supplier for the detected object to use
   * @param drive The drivetrain
   */
  public DriveToCoral(Supplier<DetectedObject> detectedObject, Drivetrain drive) {
    super(drive,
      constantUpdate
        ? () -> getPose(detectedObject)
        : () -> tempPose);
    objectSupplier = detectedObject;
    updateTarget = constantUpdate;
  }

  public static Pose2d getPose(Supplier<DetectedObject> supplier){
    DetectedObject object = supplier.get();
    if(object == null) return null;
    return new Pose2d(object.pose.toPose2d().getTranslation(), new Rotation2d(object.getAngle()+Math.PI/2));
  }

  @Override
  public void initialize(){
    // Set the static variable so the super class has access to it
    if(constantUpdate){
      tempPose = getPose(objectSupplier);
    }
    super.initialize();
  }

  @Override
  public void execute(){
    // Set the static variable so the super class has access to it
    if(constantUpdate){
      tempPose = getPose(objectSupplier);
    }
    super.execute();
  }
}
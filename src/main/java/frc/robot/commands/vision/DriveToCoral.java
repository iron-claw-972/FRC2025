package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.DetectedObject;

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
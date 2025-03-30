package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.constants.swerve.DriveConstants;
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
        ? () -> getPose(detectedObject, drive)
        : () -> tempPose);
    objectSupplier = detectedObject;
    updateTarget = constantUpdate;
    // throw new RuntimeException();
  }

  public static Pose2d getPose(Supplier<DetectedObject> supplier, Drivetrain drive){
    DetectedObject object = supplier.get();
    if(object == null) return null;
    Rotation2d rotation = new Rotation2d(MathUtil.angleModulus(object.getAngle()+Math.PI/2));
    Translation2d objectTranslation = object.pose.toPose2d().getTranslation();
    Translation2d diff = objectTranslation.minus(drive.getPose().getTranslation());
    Translation2d translation = objectTranslation.minus(diff.times(DriveConstants.ROBOT_WIDTH_WITH_BUMPERS/2/diff.getNorm()));
    return new Pose2d(translation, rotation);
  }

  @Override
  public void initialize(){
    // Set the static variable so the super class has access to it
    // if(constantUpdate){
      tempPose = getPose(objectSupplier, drive);
    // }
    super.initialize();
  }

  @Override
  public void execute(){
    //System.out.println("AAAAAAAAAAAAA:" + getPose(objectSupplier, drive));
    // Set the static variable so the super class has access to it
    if(constantUpdate){
      tempPose = getPose(objectSupplier, drive);
    }
    super.execute();
  }
}
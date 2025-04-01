package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.DetectedObject;
import frc.robot.util.Vision.DetectedObject.ObjectType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Moves toward the detected object
 * <p>Only works with the front camera
 */
public class DriveToCoral extends DriveToPose {
  private static boolean constantUpdate = true;
  private static int ticksSinceLastObject;
  private static DetectedObject cachedObject;

  /**
   * Moves toward the detected object
   * @param detectedObject The supplier for the detected object to use
   * @param drive The drivetrain
   */
  public DriveToCoral(Supplier<DetectedObject> detectedObject, Drivetrain drive) {
    super(drive,
      () -> getPose(detectedObject, drive)
    );
    updateTarget = constantUpdate;
    SmartDashboard.putBoolean("constantUpdate", constantUpdate);
  }

  @Override public void initialize() {
    cachedObject = null;
    ticksSinceLastObject = 0;
    super.initialize();
  }

  public static Pose2d getPose(Supplier<DetectedObject> supplier, Drivetrain drive){
    DetectedObject object = supplier.get();
    if(object == null || object.type != ObjectType.CORAL) {
      if (ticksSinceLastObject <= VisionConstants.MAX_EMPTY_TICKS && cachedObject != null) {
        object = cachedObject;
      } else {
        return null;
      }
      ticksSinceLastObject++;
    } else {
      ticksSinceLastObject = 0;
      cachedObject = object;
    }
    Rotation2d rotation = new Rotation2d(MathUtil.angleModulus(object.getAngle()+Math.PI/2));
    Translation2d objectTranslation = object.pose.toPose2d().getTranslation();
    Translation2d diff = objectTranslation.minus(drive.getPose().getTranslation());
    Translation2d translation = objectTranslation.minus(diff.times(DriveConstants.ROBOT_WIDTH_WITH_BUMPERS/2/diff.getNorm()));
    return new Pose2d(translation, rotation);
  }
}
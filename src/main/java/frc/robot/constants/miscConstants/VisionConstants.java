package frc.robot.constants.miscConstants;
/**
 * Container class for vision constants.
 */

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.swerve.DriveConstants;

public class VisionConstants {
  /**
   * If April tag vision is enabled on the robot
   */
  public static final boolean ENABLED = true;

  /**
   * If object detection should be enabled
   */
  public static final boolean OBJECT_DETECTION_ENABLED = false;

  /** If odometry should be updated using vision during auto */
  public static final boolean ENABLED_AUTO = true;

  /** If odometry should be updated using vision while running the GoToPose and GoToPosePID commands in teleop */
  public static final boolean ENABLED_GO_TO_POSE = true;

  /** If vision should be simulated */
  public static final boolean ENABLED_SIM = false;

  /** If vision should only return values if it can see 2 good targets */
  public static final boolean ONLY_USE_2_TAGS = false;

  /** PoseStrategy to use in pose estimation */
  public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  /** Fallback PoseStrategy if MultiTag doesn't work */
  public static final PoseStrategy MULTITAG_FALLBACK_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;

  /** Any April tags we always want to ignore. To ignore a tag, put its id in this array. */
  public static final int[] TAGS_TO_IGNORE = {};

  /** If vision should use manual calculations */
  public static final boolean USE_MANUAL_CALCULATIONS = false;

  /**
   * The number to multiply the distance to the April tag by.
   * <p>
   * Only affects manual calculations.
   * <p>
   * To find this, set it to 1 and measure the actual distance and the calculated distance.
   */
  public static final double DISTANCE_SCALE = 0.8;

  /**
   * The standard deviations to use for the vision
   */
  public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(
    0.007340, // x in meters (default=0.9)
    0.00571, // y in meters (default=0.9)
    0.9  // heading in radians. The gyroscope is very accurate, so as long as it is reset correctly it is unnecessary to correct it with vision
  );

  /**
   * The highest ambiguity to use. Ambiguities higher than this will be ignored.
   * <p>
   * Only affects calculations using PhotonVision, not manual calculations.
   */
  public static final double HIGHEST_AMBIGUITY = 0.2;

  // Speaker poses
  /**
   * The Blue Alliance speaker pose.
   * <p>
   * The game manual says the lower lip of the speaker opening is at 6 feet 6 inches (78 inches).
   * The hood comes into the arena 1 foot 6 inches (18 inches) and is at 6 feet 10 7/8 inches (82.875).
   * So the midpoint of the opening is at the average of the heights (approx 80.5 inches but really 80.4375) and
   * the average of the depths (9 inches).
   * <p>
   * April Tag ID 7 is centered on the Blue Alliance wall below the speaker opening.
   */
  public static final Pose3d BLUE_SPEAKER_POSE = new Pose3d(
    FieldConstants.APRIL_TAGS.get(6).pose.getX() + Units.inchesToMeters(9),
    FieldConstants.APRIL_TAGS.get(6).pose.getY(),
    Units.inchesToMeters(80.5),
    new Rotation3d(0, 0, 0)
  );
  /**
   * The Red Alliance speaker pose.
   * <p>
   * The game manual says the lower lip of the speaker opening is at 6 feet 6 inches (78 inches).
   * The hood comes into the arena 1 foot 6 inches (18 inches) and is at 6 feet 10 7/8 inches (82.875).
   * So the midpoint of the opening is at the average of the heights (approx 80.5 inches but really 80.4375) and
   * the average of the depths (9 inches).
   * <p>
   * April Tag ID 4 is centered on the Red Alliance wall below the speaker opening.
   */
  public static final Pose3d RED_SPEAKER_POSE = new Pose3d(
    FieldConstants.APRIL_TAGS.get(3).pose.getX() - Units.inchesToMeters(9),
    BLUE_SPEAKER_POSE.getY(),
    BLUE_SPEAKER_POSE.getZ(),
    BLUE_SPEAKER_POSE.getRotation().rotateBy(new Rotation3d(0, 0, Math.PI))
  );

  /**
   * Random distance of 2 meters.
   * <p>
   * This value is only referenced on this page, so it could be private.
   */
  public static final double AMP_DISTANCE = 2;

  // The amp poses to align to
  /**
   * The Blue Alliance Amplifier Pose2d to score a note in the amp.
   * <p>
   * This pose sets the X coordinate at the center of the amp, and
   * the Y coordinate away from the amp by half the **width** of the robot + 5 inches.
   * <p>
   * TODO: What is going on? Is the 5 inches just to press into the amp?
   * <p>
   * Maybe the 5 inches is for the intake on the competition robot,
   * but why is it being added here? That moves the robot 5 inches into the amp!
   * It should move the robot negative 5 inches to compensate for the intake.
   * However, the stern of the robot should be against the amp (we shoot from the stern),
   * and that should be half the bumper width away from the robot center;
   * the intake's additional length on the bow should not come into play.
   * <p>
   * The robot orientation has the bow facing down so the shooter will eject into the amp.
   * <p>
   * April Tag ID 6 is on the amplifier.
   */
  public static final Pose2d BLUE_AMP_POSE = new Pose2d(
    FieldConstants.APRIL_TAGS.get(5).pose.getX(),
    FieldConstants.APRIL_TAGS.get(5).pose.getY() - DriveConstants.kRobotWidthWithBumpers/2 + Units.inchesToMeters(5),
    new Rotation2d(-Math.PI/2)
  );
  /**
   * The Blue Alliance Amplifier Pose2d that is an AMP_DISTANCE away from scoring.
   * <p>
   * Not used.
   */
  public static final Pose2d BLUE_AMP_POSE_2 = new Pose2d(
    BLUE_AMP_POSE.getX(),
    BLUE_AMP_POSE.getY() - AMP_DISTANCE,
    BLUE_AMP_POSE.getRotation()
  );
  /**
   * The Blue Alliance Amplifier Pose2d that is two AMP_DISTANCEs away from scoring.
   * <p>
   * Not used.
   */
  public static final Pose2d BLUE_AMP_POSE_3 = new Pose2d(
    BLUE_AMP_POSE.getX(),
    BLUE_AMP_POSE.getY() - 2*AMP_DISTANCE,
    BLUE_AMP_POSE.getRotation()
  );

  /**
   * The Red Alliance Amplifier Pose2d to score a note in the amp.
   * <p>
   * This pose sets the X coordinate at the center of the amp, and
   * the Y coordinate away from the amp by half the length of the robot + 5 inches.
   * <p>
   * The robot orientation has the bow facing down so the shooter will eject into the amp.
   * <p>
   * April Tag ID 5 is on the Red amplifier.
   */
  public static final Pose2d RED_AMP_POSE = new Pose2d(
    FieldConstants.APRIL_TAGS.get(4).pose.getX(),
    BLUE_AMP_POSE.getY(),
    BLUE_AMP_POSE.getRotation()
  );
  /**
   * The Red Alliance Amplifier Pose2d that is an AMP_DISTANCE away from scoring.
   * <p>
   * Not used.
   */
  public static final Pose2d RED_AMP_POSE_2 = new Pose2d(
    RED_AMP_POSE.getX(),
    RED_AMP_POSE.getY() - AMP_DISTANCE,
    RED_AMP_POSE.getRotation()
  );
  /**
   * The Red Alliance Amplifier Pose2d that is 2 AMP_DISTANCEs away from scoring.
   * <p>
   * Not used.
   */
  public static final Pose2d RED_AMP_POSE_3 = new Pose2d(
    RED_AMP_POSE.getX(),
    RED_AMP_POSE.getY() - 2*AMP_DISTANCE,
    RED_AMP_POSE.getRotation()
  );

  /** How close we have to get to the amp before scoring in it (meters and radians) */
  public static final double AMP_TOLERANCE_DISTANCE = 0.1;
  public static final double AMP_TOLERANCE_ANGLE = Units.degreesToRadians(15);

  // The podium poses to align to
  public static final Pose2d BLUE_PODIUM_POSE = new Pose2d(
    FieldConstants.APRIL_TAGS.get(13).pose.getX() - Units.inchesToMeters(82.75) - DriveConstants.kRobotWidthWithBumpers/2,
    FieldConstants.APRIL_TAGS.get(13).pose.getY(),
    new Rotation2d(0)
  );
  public static final Pose2d RED_PODIUM_POSE = new Pose2d(
    FieldConstants.APRIL_TAGS.get(12).pose.getX() + Units.inchesToMeters(82.75) + DriveConstants.kRobotWidthWithBumpers/2,
    BLUE_PODIUM_POSE.getY(),
    new Rotation2d(Math.PI).minus(BLUE_PODIUM_POSE.getRotation())
  );

  public enum CHAIN_POSES{
    RED_LEFT(FieldConstants.APRIL_TAGS.get(10)),
    RED_RIGHT(FieldConstants.APRIL_TAGS.get(11)),
    RED_CENTER(FieldConstants.APRIL_TAGS.get(12)),
    BLUE_CENTER(FieldConstants.APRIL_TAGS.get(13)),
    BLUE_LEFT(FieldConstants.APRIL_TAGS.get(14)),
    BLUE_RIGHT(FieldConstants.APRIL_TAGS.get(15));

    private double dist1 = Units.inchesToMeters(50);
    private double dist2 = Units.inchesToMeters(9);
    public final Pose2d pose1;
    public final Pose2d pose2;
    private CHAIN_POSES(AprilTag tag){
      pose1 = tag.pose.toPose2d().plus(new Transform2d(new Translation2d(dist1, 0/*tag.pose.toPose2d().getRotation()*/), new Rotation2d(Math.PI)));
      pose2 = tag.pose.toPose2d().plus(new Transform2d(new Translation2d(dist2, 0/*tag.pose.toPose2d().getRotation()*/), new Rotation2d(Math.PI)));
    }
  }

  // April Tag 7 is centered on the Blue Speaker
  public static final Pose2d BLUE_SUBWOOFER_CENTER = new Pose2d(
    FieldConstants.APRIL_TAGS.get(6).pose.getX()+Units.inchesToMeters(53.904),
    FieldConstants.APRIL_TAGS.get(6).pose.getY(),
    new Rotation2d()
  );
  public static final Pose2d BLUE_SUBWOOFER_LEFT = new Pose2d(
    FieldConstants.APRIL_TAGS.get(6).pose.getX()+Units.inchesToMeters(27.562),
    FieldConstants.APRIL_TAGS.get(6).pose.getY()+Units.inchesToMeters(45.292),
    Rotation2d.fromDegrees(60)
  );
  public static final Pose2d BLUE_SUBWOOFER_RIGHT = new Pose2d(
    FieldConstants.APRIL_TAGS.get(6).pose.getX()+Units.inchesToMeters(27.562),
    FieldConstants.APRIL_TAGS.get(6).pose.getY()-Units.inchesToMeters(45.292),
    Rotation2d.fromDegrees(-60)
  );

  // April Tag 4 is centered on the Red Speaker
  public static final Pose2d RED_SUBWOOFER_CENTER = new Pose2d(
    FieldConstants.APRIL_TAGS.get(3).pose.getX()-Units.inchesToMeters(53.904),
    BLUE_SUBWOOFER_CENTER.getY(),
    new Rotation2d(Math.PI-BLUE_SUBWOOFER_CENTER.getRotation().getRadians())
  );
  public static final Pose2d RED_SUBWOOFER_LEFT = new Pose2d(
    FieldConstants.APRIL_TAGS.get(3).pose.getX()-Units.inchesToMeters(27.562),
    BLUE_SUBWOOFER_RIGHT.getY(),
    new Rotation2d(Math.PI-BLUE_SUBWOOFER_RIGHT.getRotation().getRadians())
  );
  public static final Pose2d RED_SUBWOOFER_RIGHT = new Pose2d(
    FieldConstants.APRIL_TAGS.get(3).pose.getX()-Units.inchesToMeters(27.562),
    BLUE_SUBWOOFER_LEFT.getY(),
    new Rotation2d(Math.PI-BLUE_SUBWOOFER_LEFT.getRotation().getRadians())
  );

  // Distance to the speaker for preset stage shooting
  private static final double distToSpeaker = 4;
  // Positions to shoot from near the stage
  public static final Pose2d BLUE_STAGE_SHOOT_POSE = new Pose2d(
    FieldConstants.APRIL_TAGS.get(6).pose.getX() - distToSpeaker,
    FieldConstants.APRIL_TAGS.get(6).pose.getY(),
    new Rotation2d(0)
  );
  public static final Pose2d RED_STAGE_SHOOT_POSE = new Pose2d(
    FieldConstants.APRIL_TAGS.get(3).pose.getX() - distToSpeaker,
    BLUE_STAGE_SHOOT_POSE.getY(),
    new Rotation2d(Math.PI).minus(BLUE_STAGE_SHOOT_POSE.getRotation())
  );

  /**
   * The camera poses
   * <p>
   * + X: Front of Robot <p>
   * + Y: Left of Robot <p>
   * + Z: Top of Robot
   */
  public static final ArrayList<Pair<String, Transform3d>> APRIL_TAG_CAMERAS = new ArrayList<Pair<String, Transform3d>>(List.of(
    new Pair<String, Transform3d>(
      "CameraPort",
      new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.917), Units.inchesToMeters(6.2), Units.inchesToMeters(18.67)),
        new Rotation3d(0, Units.degreesToRadians(-20), Math.PI+Units.degreesToRadians(15))
      )
    ),
    new Pair<String, Transform3d>(
      "CameraStarboard",
      new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.917), Units.inchesToMeters(-6.2), Units.inchesToMeters(18.67)),
        new Rotation3d(0, Units.degreesToRadians(-20), Math.PI-Units.degreesToRadians(15))
      ))
    )
  );

  /**
   * The transformations from the robot to object detection cameras
   */
  public static final ArrayList<Transform3d> OBJECT_DETECTION_CAMERAS = new ArrayList<>(List.of(
    new Transform3d(
      new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(24)),
      new Rotation3d(0, Units.degreesToRadians(20), 0))
  ));
}

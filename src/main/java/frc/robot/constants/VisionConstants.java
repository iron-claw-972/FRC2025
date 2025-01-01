package frc.robot.constants;
/**
 * Container class for vision constants.
 */

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

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
  public static final boolean ENABLED_SIM = true;

  /** If vision should only return values if it can see 2 good targets */
  public static final boolean ONLY_USE_2_TAGS = true;

  /** PoseStrategy to use in pose estimation */
  public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  /** Fallback PoseStrategy if MultiTag doesn't work */
  public static final PoseStrategy MULTITAG_FALLBACK_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;

  /** Any April tags we always want to ignore. To ignore a tag, put its id in this array. */
  public static final int[] TAGS_TO_IGNORE = {};

  /** If multiple cameras return different poses, they will be ignored if the difference between them is greater than this */
  public static final double MAX_POSE_DIFFERENCE = 0.3;

  /** If vision should use manual calculations */
  public static final boolean USE_MANUAL_CALCULATIONS = false;

  /**
   * The number to multiply the distance to the April tag by.
   * <p>
   * Only affects manual calculations.
   * <p>
   * To find this, set it to 1 and measure the actual distance and the calculated distance.
   */
  public static final double DISTANCE_SCALE = 1;

  /**
   * The standard deviations to use for the vision
   */
  public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(
    0.9, // x in meters (default=0.9)
    0.9, // y in meters (default=0.9)
    0.9  // heading in radians. The gyroscope is very accurate, so as long as it is reset correctly it is unnecessary to correct it with vision
  );

  /**
   * The highest ambiguity to use. Ambiguities higher than this will be ignored.
   * <p>
   * Only affects calculations using PhotonVision, not manual calculations.
   */
  public static final double HIGHEST_AMBIGUITY = 0.2;

  /**
   * The camera poses
   * <p>
   * Everything is in meters and radians <p>
   * 0 for all numbers is center of the robot, on the ground, looking straight toward the front <p>
   * + X: Front of Robot <p>
   * + Y: Left of Robot <p>
   * + Z: Top of Robot <p>
   * + Pitch: Down <p>
   * + Yaw: Counterclockwise
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

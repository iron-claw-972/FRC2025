package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.swerve.DriveConstants;

/**
 * Container class for vision constants.
 */
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

  /**
   * If odometry should be updated using vision while running the GoToPose and
   * GoToPosePID commands in teleop
   */
  public static final boolean ENABLED_GO_TO_POSE = true;

  /** If vision should be simulated */
  public static final boolean ENABLED_SIM = true;

  /** If vision should only return values if it can see 2 good targets */
  public static final boolean ONLY_USE_2_TAGS = true;

  /** PoseStrategy to use in pose estimation */
  public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  /** Fallback PoseStrategy if MultiTag doesn't work */
  public static final PoseStrategy MULTITAG_FALLBACK_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;

  /**
   * Any April tags we always want to ignore. To ignore a tag, put its id in this
   * array.
   */
  public static final int[] TAGS_TO_IGNORE = {};

  /**
   * If multiple cameras return different poses, they will be ignored if the
   * difference between them is greater than this
   */
  public static final double MAX_POSE_DIFFERENCE = 0.3;

  /** If vision should use manual calculations */
  public static final boolean USE_MANUAL_CALCULATIONS = false;

  // <ol start="0"> did not work
  /**
   * Which version of driver assist to use. This would be an enum, except there is
   * no short and descriptive name for all of these.
   * <p>
   * The options are:
   * <p>
   * 0: Disable driver assist
   * <p>
   * 1: Completely remove the component of the driver's input that is not toward
   * the object
   * <p>
   * 2: Interpolate between the next achievable driver speed and a speed
   * calculated using trapezoid profiles
   * <p>
   * 3-5: Add a speed perpendicular to the driver input; there are 3 similar but
   * different calculations for this
   */
  public static final int DRIVER_ASSIST_MODE = 1;

  /**
   * The number to multiply the distance to the April tag by.
   * <p>
   * Only affects manual calculations.
   * <p>
   * To find this, set it to 1 and measure the actual distance and the calculated
   * distance.
   * <p>
   * This should not be needed, and it is only here because it improved the
   * accuracy of vision in the 2023 fall semester
   */
  public static final double DISTANCE_SCALE = 1;

  /**
   * The standard deviations to use for the vision
   */
  public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(
      0.9, // x in meters (default=0.9)
      0.9, // y in meters (default=0.9)
      0.9 // heading in radians. The gyroscope is very accurate, so as long as it is reset
          // correctly it is unnecessary to correct it with vision
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
   * Everything is in meters and radians
   * <p>
   * 0 for all numbers is center of the robot, on the ground, looking straight
   * toward the front
   * <p>
   * + X: Front of Robot
   * <p>
   * + Y: Left of Robot
   * <p>
   * + Z: Top of Robot
   * <p>
   * + Pitch: Down
   * <p>
   * + Yaw: Counterclockwise
   */

  public static final ArrayList<Pair<String, Transform3d>> APRIL_TAG_CAMERAS = new ArrayList<Pair<String, Transform3d>>(
      List.of(
          new Pair<String, Transform3d>(
              "CameraPort",
              new Transform3d(
                  new Translation3d(Units.inchesToMeters(-11.917), Units.inchesToMeters(6.2),
                      Units.inchesToMeters(18.67)),
                  new Rotation3d(0, Units.degreesToRadians(-20), Math.PI + Units.degreesToRadians(15)))),
          new Pair<String, Transform3d>(
              "CameraStarboard",
              new Transform3d(
                  new Translation3d(Units.inchesToMeters(-11.917), Units.inchesToMeters(-6.2),
                      Units.inchesToMeters(18.67)),
                  new Rotation3d(0, Units.degreesToRadians(-20), Math.PI - Units.degreesToRadians(15))))));

  /**
   * The transformations from the robot to object detection cameras
   */
  public static final ArrayList<Transform3d> OBJECT_DETECTION_CAMERAS = new ArrayList<>(List.of(
      new Transform3d(
          new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(24)),
          new Rotation3d(0, Units.degreesToRadians(20), 0))));

  // Poses to potentially align to

  public static final Pose2d RED_CORAL_STATION_LEFT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(0).pose.getX(),
      FieldConstants.APRIL_TAGS.get(0).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d RED_CORAL_STATION_RIGHT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(1).pose.getX(),
      FieldConstants.APRIL_TAGS.get(1).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_CORAL_STATION_LEFT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(12).pose.getX(),
      FieldConstants.APRIL_TAGS.get(12).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_CORAL_STATION_RIGHT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(11).pose.getX(),
      FieldConstants.APRIL_TAGS.get(11).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));





  public static final Pose2d BLUE_PROCESSOR_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(2).pose.getX(),
      FieldConstants.APRIL_TAGS.get(2).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d RED_PROCESSOR_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(15).pose.getX(),
      FieldConstants.APRIL_TAGS.get(15).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));





  public static final Pose2d BLUE_LEFT_CAGE_POSE = new Pose2d(
      8.775,
      FieldConstants.APRIL_TAGS.get(13).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2 + Units.feetToMeters(6),
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_MIDDLE_CAGE_POSE = new Pose2d(
      8.775,
      FieldConstants.APRIL_TAGS.get(13).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_RIGHT_CAGE_POSE = new Pose2d(
      8.775,
      FieldConstants.APRIL_TAGS.get(13).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2 - Units.feetToMeters(6),
      new Rotation2d(-Math.PI / 2));





  public static final Pose2d RED_LEFT_CAGE_POSE = new Pose2d(
      8.775,
      FieldConstants.APRIL_TAGS.get(14).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2 + Units.feetToMeters(6),
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d RED_MIDDLE_CAGE_POSE = new Pose2d(
      8.775,
      FieldConstants.APRIL_TAGS.get(14).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d RED_RIGHT_CAGE_POSE = new Pose2d(
      8.775,
      FieldConstants.APRIL_TAGS.get(14).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2 - Units.feetToMeters(6),
      new Rotation2d(-Math.PI / 2));





  public static final Pose2d BLUE_BRANCH_17_LEFT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(16).pose.getX() + Units.inchesToMeters(2.31), // approximate x - distance from reef base to center of branch
      FieldConstants.APRIL_TAGS.get(16).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2 + Units.inchesToMeters(5.14), // approximate y - distance from reef base to center of branch
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_BRANCH_17_RIGHT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(16).pose.getX(),
      FieldConstants.APRIL_TAGS.get(16).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_BRANCH_18_LEFT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(17).pose.getX(),
      FieldConstants.APRIL_TAGS.get(17).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_BRANCH_18_RIGHT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(17).pose.getX(),
      FieldConstants.APRIL_TAGS.get(17).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_BRANCH_19_LEFT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(18).pose.getX(),
      FieldConstants.APRIL_TAGS.get(18).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_BRANCH_19_RIGHT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(18).pose.getX(),
      FieldConstants.APRIL_TAGS.get(18).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_BRANCH_20_LEFT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(19).pose.getX(),
      FieldConstants.APRIL_TAGS.get(19).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_BRANCH_20_RIGHT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(19).pose.getX(),
      FieldConstants.APRIL_TAGS.get(19).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_BRANCH_21_LEFT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(20).pose.getX(),
      FieldConstants.APRIL_TAGS.get(20).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_BRANCH_21_RIGHT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(20).pose.getX(),
      FieldConstants.APRIL_TAGS.get(20).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_BRANCH_22_LEFT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(21).pose.getX(),
      FieldConstants.APRIL_TAGS.get(21).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));

  public static final Pose2d BLUE_BRANCH_22_RIGHT_POSE = new Pose2d(
      FieldConstants.APRIL_TAGS.get(21).pose.getX(),
      FieldConstants.APRIL_TAGS.get(21).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
      new Rotation2d(-Math.PI / 2));
}

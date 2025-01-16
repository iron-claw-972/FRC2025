package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
                                    new Rotation3d(0, Units.degreesToRadians(-20),
                                            Math.PI + Units.degreesToRadians(15)))),
                    new Pair<String, Transform3d>(
                            "CameraStarboard",
                            new Transform3d(
                                    new Translation3d(Units.inchesToMeters(-11.917), Units.inchesToMeters(-6.2),
                                            Units.inchesToMeters(18.67)),
                                    new Rotation3d(0, Units.degreesToRadians(-20),
                                            Math.PI - Units.degreesToRadians(15))))));

    /**
     * The transformations from the robot to object detection cameras
     */
    public static final ArrayList<Transform3d> OBJECT_DETECTION_CAMERAS = new ArrayList<>(List.of(
            new Transform3d(
                    new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(24)),
                    new Rotation3d(0, Units.degreesToRadians(20), 0))));

    // Poses to potentially align to

    public static final Pose2d RED_CORAL_STATION_LEFT_POSE = new Pose2d(
            FieldConstants.APRIL_TAGS.get(0).pose.getX() +
                    (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2) * Math.cos(Units.degreesToRadians(126)),
            FieldConstants.APRIL_TAGS.get(0).pose.getY() +
                    (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2) * Math.sin(Units.degreesToRadians(126)),
            new Rotation2d(Units.degreesToRadians(126)));

    public static final Pose2d RED_CORAL_STATION_RIGHT_POSE = new Pose2d(
            FieldConstants.APRIL_TAGS.get(1).pose.getX() +
                    (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2) * Math.cos(Units.degreesToRadians(234)),
            FieldConstants.APRIL_TAGS.get(1).pose.getY() +
                    (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2) * Math.sin(Units.degreesToRadians(234)),
            new Rotation2d(Units.degreesToRadians(234)));

    public static final Pose2d BLUE_CORAL_STATION_LEFT_POSE = new Pose2d(
            FieldConstants.APRIL_TAGS.get(12).pose.getX() +
                    (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2) * Math.cos(Units.degreesToRadians(306)),
            FieldConstants.APRIL_TAGS.get(12).pose.getY() +
                    (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2) * Math.sin(Units.degreesToRadians(306)),
            new Rotation2d(Units.degreesToRadians(306)));

    public static final Pose2d BLUE_CORAL_STATION_RIGHT_POSE = new Pose2d(
            FieldConstants.APRIL_TAGS.get(11).pose.getX() +
                    (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2) * Math.cos(Units.degreesToRadians(54)),
            FieldConstants.APRIL_TAGS.get(11).pose.getY() +
                    (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2) * Math.sin(Units.degreesToRadians(54)),
            new Rotation2d(Units.degreesToRadians(54)));

    public static final Pose2d RED_PROCESSOR_POSE = new Pose2d(
            FieldConstants.APRIL_TAGS.get(2).pose.getX(),
            FieldConstants.APRIL_TAGS.get(2).pose.getY() - DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
            new Rotation2d(Math.PI / 2));

    public static final Pose2d BLUE_PROCESSOR_POSE = new Pose2d(
            FieldConstants.APRIL_TAGS.get(15).pose.getX(),
            FieldConstants.APRIL_TAGS.get(15).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
            new Rotation2d(-Math.PI / 2));

    public static final Pose2d BLUE_CAGE_LEFT_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.FIELD_WIDTH / 2 + Units.inchesToMeters(1019.0 / 8),
            new Rotation2d());

    public static final Pose2d BLUE_CAGE_MIDDLE_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.FIELD_WIDTH / 2 + Units.inchesToMeters(675.0 / 8),
            new Rotation2d());

    public static final Pose2d BLUE_CAGE_RIGHT_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.FIELD_WIDTH / 2 + Units.inchesToMeters(41.5),
            new Rotation2d());

    public static final Pose2d RED_CAGE_RIGHT_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.FIELD_WIDTH / 2 + Units.inchesToMeters(-41.5),
            new Rotation2d(Math.PI));

    public static final Pose2d RED_CAGE_MIDDLE_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.FIELD_WIDTH / 2 - Units.inchesToMeters(-675.0 / 8),
            new Rotation2d(Math.PI));

    public static final Pose2d RED_CAGE_LEFT_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.FIELD_WIDTH / 2 - Units.inchesToMeters(-1019.0 / 8),
            new Rotation2d(Math.PI));

    /**
     * Stores all of the alignment poses for both reefs.
     * Each branch is in the format (RED or BLUE)_BRANCH_(tag ID)_(LEFT or RIGHT)
     * Left and right refer to the position of the branch when looking directly at the AprilTag
     */
    public enum REEF {
        RED_BRANCH_6_LEFT(5, 0.1308, .0587),
        RED_BRANCH_6_RIGHT(5, -0.1308, .0587),
        RED_BRANCH_7_LEFT(6, 0.1308, .0587),
        RED_BRANCH_7_RIGHT(6, -0.1308, .0587),
        RED_BRANCH_8_LEFT(7, 0.1308, .0587),
        RED_BRANCH_8_RIGHT(7, -0.1308, .0587),
        RED_BRANCH_9_LEFT(8, 0.1308, .0587),
        RED_BRANCH_9_RIGHT(8, -0.1308, .0587),
        RED_BRANCH_10_LEFT(9, 0.1308, .0587),
        RED_BRANCH_10_RIGHT(9, -0.1308, .0587),
        RED_BRANCH_11_LEFT(10, 0.1308, .0587),
        RED_BRANCH_11_RIGHT(10, -0.1308, .0587),

        BLUE_BRANCH_17_LEFT(16, 0.1308, .0587),
        BLUE_BRANCH_17_RIGHT(16, -0.1308, .0587),
        BLUE_BRANCH_18_LEFT(17, 0.1308, .0587),
        BLUE_BRANCH_18_RIGHT(17, -0.1308, .0587),
        BLUE_BRANCH_19_LEFT(18, 0.1308, .0587),
        BLUE_BRANCH_19_RIGHT(18, -0.1308, .0587),
        BLUE_BRANCH_20_LEFT(19, 0.1308, .0587),
        BLUE_BRANCH_20_RIGHT(19, -0.1308, .0587),
        BLUE_BRANCH_21_LEFT(20, 0.1308, .0587),
        BLUE_BRANCH_21_RIGHT(20, -0.1308, .0587),
        BLUE_BRANCH_22_LEFT(21, 0.1308, .0587),
        BLUE_BRANCH_22_RIGHT(21, -0.1308, .0587);

        /**
         * The pose to align to for scoring on this branch
         */
        public final Pose2d pose;
        /**
         * The ID of the AprilTag on the smae side of hte reef ast his branch
         */
        public final int aprilTagId;
        private final int aprilTagIndex;
        /**
         * The horizontal (parallel to the closest face of the reef) distance, in meters, between the AprilTag and branch
         */
        public final double xOffset;
        /**
         * The y (normal to the closest face of the reef) distance, in meters, between the AprilTag and branch
         */
        public final double yOffset;

        private REEF(int aprilTagIndex, double xOffset, double yOffset) {
            this.aprilTagIndex = aprilTagIndex;
            aprilTagId = aprilTagIndex+1;
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            pose = getPose();
        }

        /**
         * Calculates the Pose2d to align to the branch based on the AprilTag's pose and
         * offsets.
         *
         * @return The calculated Pose2d for this reef branch.
         */
        private Pose2d getPose() {
            Pose3d basePose3d = FieldConstants.APRIL_TAGS.get(aprilTagIndex).pose;
            double adjustedYOffset = DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2.0;

            // Apply both X and Y offsets to calculate the reef branch pose
            Transform3d transform = new Transform3d(adjustedYOffset, -xOffset, 0, new Rotation3d(0, 0, Math.PI));

            Pose3d branchPose3d = basePose3d.plus(transform);

            // Convert the calculated branch Pose3d to Pose2d
            return branchPose3d.toPose2d();
        }

        /**
         * Finds the appropriate ReefBranch based on the April tag ID and whether the
         * pose is left or right.
         *
         * @param aprilTagId The ID of the April tag.
         * @param isLeftPose True if the pose is left, false if it's right.
         * @return The matching ReefBranch, or null if no match is found.
         */
        public static REEF fromAprilTagIdAndPose(int aprilTagId, boolean isLeftPose) {
            for (REEF branch : values()) {
                if (branch.aprilTagId == aprilTagId &&
                        ((isLeftPose && branch.xOffset > 0) || (!isLeftPose && branch.xOffset < 0))) {
                    return branch;
                }
            }
            return null;
        }
    }
}
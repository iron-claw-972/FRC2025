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
            FieldConstants.APRIL_TAGS.get(2).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
            new Rotation2d(Math.PI / 2));

    public static final Pose2d BLUE_PROCESSOR_POSE = new Pose2d(
            FieldConstants.APRIL_TAGS.get(15).pose.getX(),
            FieldConstants.APRIL_TAGS.get(15).pose.getY() + DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
            new Rotation2d(-Math.PI / 2));

    public static final Pose2d BLUE_LEFT_CAGE_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.APRIL_TAGS.get(13).pose.getY() + Units.feetToMeters(6),
            new Rotation2d(-Math.PI / 2));

    public static final Pose2d BLUE_MIDDLE_CAGE_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.APRIL_TAGS.get(13).pose.getY(),
            new Rotation2d(-Math.PI / 2));

    public static final Pose2d BLUE_RIGHT_CAGE_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.APRIL_TAGS.get(13).pose.getY() - Units.feetToMeters(6),
            new Rotation2d(-Math.PI / 2));

    public static final Pose2d RED_LEFT_CAGE_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.APRIL_TAGS.get(14).pose.getY() + Units.feetToMeters(6),
            new Rotation2d(-Math.PI / 2));

    public static final Pose2d RED_MIDDLE_CAGE_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.APRIL_TAGS.get(14).pose.getY(),
            new Rotation2d(-Math.PI / 2));

    public static final Pose2d RED_RIGHT_CAGE_POSE = new Pose2d(
            FieldConstants.FIELD_LENGTH / 2,
            FieldConstants.APRIL_TAGS.get(14).pose.getY() - Units.feetToMeters(6),
            new Rotation2d(-Math.PI / 2));

    public enum BLUE_REEF {
        BLUE_BRANCH_17_LPOSE(16, 0.1308, .0587),
        BLUE_BRANCH_17_RPOSE(16, -0.1308, .0587),
        BLUE_BRANCH_18_LPOSE(17, 0.1308, .0587),
        BLUE_BRANCH_18_RPOSE(17, -0.1308, .0587),
        BLUE_BRANCH_19_LPOSE(18, 0.1308, .0587),
        BLUE_BRANCH_19_RPOSE(18, -0.1308, .0587),
        BLUE_BRANCH_20_LPOSE(19, 0.1308, .0587),
        BLUE_BRANCH_20_RPOSE(19, -0.1308, .0587),
        BLUE_BRANCH_21_LPOSE(20, 0.1308, .0587),
        BLUE_BRANCH_21_RPOSE(20, -0.1308, .0587),
        BLUE_BRANCH_22_LPOSE(21, 0.1308, .0587),
        BLUE_BRANCH_22_RPOSE(21, -0.1308, .0587);

        private final int aprilTagId;
        private final double xOffset;
        private final double yOffset;

        BLUE_REEF(int aprilTagId, double xOffset, double yOffset) {
            this.aprilTagId = aprilTagId;
            this.xOffset = xOffset;
            this.yOffset = yOffset;
        }

        /**
         * Calculates the Pose2d for the branch based on the April tag's base pose and
         * offsets.
         *
         * @param basePose3d The Pose3d of the April tag base.
         * @return The calculated Pose2d for this reef branch.
         */
        public Pose2d getPose(Pose3d basePose3d) {
            // Convert the April tag base Pose3d to Pose2d
            Pose2d basePose2d = new Pose2d(
                    basePose3d.getX(),
                    basePose3d.getY(),
                    basePose3d.getRotation().toRotation2d());

            double adjustedYOffset = yOffset + (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2.0);

            // Apply both X and Y offsets to calculate the reef branch pose
            Translation3d offsetTranslation = new Translation3d(xOffset, adjustedYOffset, 0)
                    .rotateBy(basePose3d.getRotation());

            Pose3d branchPose3d = basePose3d.plus(new Transform3d(offsetTranslation, new Rotation3d()));

            // Convert the calculated branch Pose3d to Pose2d
            return new Pose2d(
                    branchPose3d.getX(),
                    branchPose3d.getY(),
                    branchPose3d.getRotation().toRotation2d());
        }

        /**
         * Finds the appropriate ReefBranch based on the April tag ID and whether the
         * pose is left or right.
         *
         * @param aprilTagId The ID of the April tag.
         * @param isLeftPose True if the pose is left, false if it's right.
         * @return The matching ReefBranch, or null if no match is found.
         */
        public static BLUE_REEF fromAprilTagIdAndPose(int aprilTagId, boolean isLeftPose) {
            for (BLUE_REEF branch : values()) {
                if (branch.aprilTagId == aprilTagId &&
                        ((isLeftPose && branch.xOffset > 0) || (!isLeftPose && branch.xOffset < 0))) {
                    return branch;
                }
            }
            return null;
        }
    }

    public enum RED_REEF {
        RED_BRANCH_6_LPOSE(5, 0.1308, .0587),
        RED_BRANCH_6_RPOSE(5, -0.1308, .0587),
        RED_BRANCH_7_LPOSE(6, 0.1308, .0587),
        RED_BRANCH_7_RPOSE(6, -0.1308, .0587),
        RED_BRANCH_8_LPOSE(7, 0.1308, .0587),
        RED_BRANCH_8_RPOSE(7, -0.1308, .0587),
        RED_BRANCH_9_LPOSE(8, 0.1308, .0587),
        RED_BRANCH_9_RPOSE(8, -0.1308, .0587),
        RED_BRANCH_10_LPOSE(9, 0.1308, .0587),
        RED_BRANCH_10_RPOSE(9, -0.1308, .0587),
        RED_BRANCH_11_LPOSE(10, 0.1308, .0587),
        RED_BRANCH_11_RPOSE(10, -0.1308, .0587);

        private final int aprilTagId;
        private final double xOffset;
        private final double yOffset;

        RED_REEF(int aprilTagId, double xOffset, double yOffset) {
            this.aprilTagId = aprilTagId;
            this.xOffset = xOffset;
            this.yOffset = yOffset;
        }

        /**
         * Calculates the Pose2d for the branch based on the April tag's base pose and
         * offsets.
         *
         * @param basePose3d The Pose3d of the April tag base.
         * @return The calculated Pose2d for this reef branch.
         */
        public Pose2d getPose(Pose3d basePose3d) {
            // Convert the April tag base Pose3d to Pose2d
            Pose2d basePose2d = new Pose2d(
                    basePose3d.getX(),
                    basePose3d.getY(),
                    basePose3d.getRotation().toRotation2d());

            // Apply both X and Y offsets to calculate the reef branch pose
            Translation3d offsetTranslation = new Translation3d(xOffset, yOffset, 0)
                    .rotateBy(basePose3d.getRotation());

            Pose3d branchPose3d = basePose3d.plus(new Transform3d(offsetTranslation, new Rotation3d()));

            // Convert the calculated branch Pose3d to Pose2d
            return new Pose2d(
                    branchPose3d.getX(),
                    branchPose3d.getY(),
                    branchPose3d.getRotation().toRotation2d());
        }

        /**
         * Finds the appropriate ReefBranch based on the April tag ID and whether the
         * pose is left or right.
         *
         * @param aprilTagId The ID of the April tag.
         * @param isLeftPose True if the pose is left, false if it's right.
         * @return The matching ReefBranch, or null if no match is found.
         */
        public static RED_REEF fromAprilTagIdAndPose(int aprilTagId, boolean isLeftPose) {
            for (RED_REEF branch : values()) {
                if (branch.aprilTagId == aprilTagId &&
                        ((isLeftPose && branch.xOffset > 0) || (!isLeftPose && branch.xOffset < 0))) {
                    return branch;
                }
            }
            return null;
        }
    }
}
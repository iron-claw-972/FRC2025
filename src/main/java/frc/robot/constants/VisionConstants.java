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
import edu.wpi.first.math.geometry.Transform2d;
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
     * If odometry should be updated using vision while running the GoToPose,
     * GoToPosePID, and DriveToPose commands in teleop
     */
    public static final boolean ENABLED_GO_TO_POSE = true;

    /** If vision should be simulated */
    public static final boolean ENABLED_SIM = false;

    /** If vision should only return values if it can see 2 good targets */
    public static final boolean ONLY_USE_2_TAGS = false;

    /** PoseStrategy to use in pose estimation */
    public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    /** Fallback PoseStrategy if MultiTag doesn't work */
    public static final PoseStrategy MULTITAG_FALLBACK_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;

    /**
     * Any April tags we always want to ignore. To ignore a tag, put its id in this
     * array.
     */
    public static final int[] TAGS_TO_IGNORE = {12, 13, 16, 1, 2, 3};

    /**
     * If multiple cameras return different poses, they will be ignored if the
     * difference between them is greater than this
     */
    public static final double MAX_POSE_DIFFERENCE = 0.2;

    /**
     * The maximum distance to the tag to use
     */
    public static final double MAX_DISTANCE = 2;

    /** If vision should use manual calculations */
    public static final boolean USE_MANUAL_CALCULATIONS = true;

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
    public static final int DRIVER_ASSIST_MODE = 5;

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
     * The standard deviations to use for vision
     */
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(
            0.3, // x in meters (default=0.9)
            0.3, // y in meters (default=0.9)
            0.9 // heading in radians. The gyroscope is very accurate, so as long as it is reset
                // correctly it is unnecessary to correct it with vision
    );

    /**
     * The standard deviations to use for vision when the wheels slip
     */
    public static final Matrix<N3, N1> VISION_STD_DEVS_2 = VecBuilder.fill(
            0.01, // x in meters (default=0.9)
            0.01, // y in meters (default=0.9)
            0.9 // heading in radians. The gyroscope is very accurate, so as long as it is reset
                // correctly it is unnecessary to correct it with vision
    );

    /**
     * The highest ambiguity to use. Ambiguities higher than this will be ignored.
     * <p>
     * Only affects calculations using PhotonVision, not manual calculations.
     */
    public static final double HIGHEST_AMBIGUITY = 0.01;

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
                        "CameraFront",
                        new Transform3d(
                                new Translation3d(Units.inchesToMeters(10.485), Units.inchesToMeters(10.217),
                                        Units.inchesToMeters(11.012)),
                                new Rotation3d(0, Units.degreesToRadians(-11),
                                        Math.PI/2 + Units.degreesToRadians(20)))),
                new Pair<String, Transform3d>(
                        "CameraBack",
                        new Transform3d(
                                new Translation3d(Units.inchesToMeters(-9.538), Units.inchesToMeters(7.474),
                                        Units.inchesToMeters(8.719)),
                                new Rotation3d(0, Units.degreesToRadians(-19.5),
                                        Math.PI/2-Units.degreesToRadians(25))))));

    /**
     * The transformations from the robot to object detection cameras
     */
    public static final ArrayList<Transform3d> OBJECT_DETECTION_CAMERAS = new ArrayList<>(List.of(
            new Transform3d(
                    new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(24)),
                    new Rotation3d(0, Units.degreesToRadians(20), 0))));

    // Poses to potentially align to
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
    * The distance between the AprilTag and algae setpoint in the directin parallel to the face of the reef
    * Positive is to the left
    */
    public static final double ALGAE_X_OFFSET = 0.02;

    /**
     * Stores all of the alignment poses for both reefs.
     * Each branch is in the format (RED or BLUE)_BRANCH_(tag ID)_(LEFT or RIGHT)
     * Each algae setpoint is in the form (RED or BLUE)_ALGAE_(tag ID)
     * Left and right refer to the position of the branch when looking directly at the AprilTag
     */
    public enum REEF {
        RED_BRANCH_6_LEFT(5, 0.1651, .0587),
        RED_BRANCH_6_RIGHT(5, -0.1651, .0587),
        RED_BRANCH_7_LEFT(6, 0.1651, .0587),
        RED_BRANCH_7_RIGHT(6, -0.1651, .0587),
        RED_BRANCH_8_LEFT(7, 0.1651, .0587),
        RED_BRANCH_8_RIGHT(7, -0.1651, .0587),
        RED_BRANCH_9_LEFT(8, 0.1651, .0587),
        RED_BRANCH_9_RIGHT(8, -0.1651, .0587),
        RED_BRANCH_10_LEFT(9, 0.1651, .0587),
        RED_BRANCH_10_RIGHT(9, -0.1651, .0587),
        RED_BRANCH_11_LEFT(10, 0.1651, .0587),
        RED_BRANCH_11_RIGHT(10, -0.1651, .0587),

        BLUE_BRANCH_17_LEFT(16, 0.1651, .0587),
        BLUE_BRANCH_17_RIGHT(16, -0.1651, .0587),
        BLUE_BRANCH_18_LEFT(17, 0.1651, .0587),
        BLUE_BRANCH_18_RIGHT(17, -0.1651, .0587),
        BLUE_BRANCH_19_LEFT(18, 0.1651, .0587),
        BLUE_BRANCH_19_RIGHT(18, -0.1651, .0587),
        BLUE_BRANCH_20_LEFT(19, 0.1651, .0587),
        BLUE_BRANCH_20_RIGHT(19, -0.1651, .0587),
        BLUE_BRANCH_21_LEFT(20, 0.1651, .0587),
        BLUE_BRANCH_21_RIGHT(20, -0.1651, .0587),
        BLUE_BRANCH_22_LEFT(21, 0.1651, .0587),
        BLUE_BRANCH_22_RIGHT(21, -0.1651, .0587),

        RED_ALGAE_6(5, ALGAE_X_OFFSET, 0, true),
        RED_ALGAE_7(6, ALGAE_X_OFFSET, 0, true),
        RED_ALGAE_8(7, ALGAE_X_OFFSET, 0, true),
        RED_ALGAE_9(8, ALGAE_X_OFFSET, 0, true),
        RED_ALGAE_10(9, ALGAE_X_OFFSET, 0, true),
        RED_ALGAE_11(10, ALGAE_X_OFFSET, 0, true),
        BLUE_ALGAE_17(16, ALGAE_X_OFFSET, 0, true),
        BLUE_ALGAE_18(17, ALGAE_X_OFFSET, 0, true),
        BLUE_ALGAE_19(18, ALGAE_X_OFFSET, 0, true),
        BLUE_ALGAE_20(19, ALGAE_X_OFFSET, 0, true),
        BLUE_ALGAE_21(20, ALGAE_X_OFFSET, 0, true),
        BLUE_ALGAE_22(21, ALGAE_X_OFFSET, 0, true);

        /**
         * The pose to align to for scoring on this branch or intaking this algae
         */
        public final Pose2d pose;
        /**
         * The pose to align to for scoring on L4, null for algae
         */
        public final Pose2d l4Pose;
        /**
         * The pose to align to for scoring on L1, null for algae
         */
        public final Pose2d l1Pose;
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

        public final boolean isAlgae;

        private REEF(int aprilTagIndex, double xOffset, double yOffset) {
            this(aprilTagIndex, xOffset, yOffset, false);
        }
        private REEF(int aprilTagIndex, double xOffset, double yOffset, boolean isAlgae) {
            this.aprilTagIndex = aprilTagIndex;
            aprilTagId = aprilTagIndex+1;
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.isAlgae = isAlgae;
            pose = getPose();
            if(isAlgae){
                l4Pose = null;
                l1Pose = null;
            }else{
                l4Pose = pose.transformBy(new Transform2d(0, Units.inchesToMeters(2), new Rotation2d()));
                l1Pose = getL1Pose();
            }
        }

        /**
         * Calculates the Pose2d to align to the branch based on the AprilTag's pose and
         * offsets.
         *
         * @return The calculated Pose2d for this reef branch.
         */
        private Pose2d getPose() {
            Pose3d basePose3d = FieldConstants.APRIL_TAGS.get(aprilTagIndex).pose;
            double adjustedYOffset = DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2.0 + (isAlgae ? 0 : Units.inchesToMeters(4.5));

            // Apply both X and Y offsets to calculate the reef branch pose
            Transform3d transform = new Transform3d(adjustedYOffset, -xOffset, 0, new Rotation3d(0, 0, 0));

            Pose3d branchPose3d = basePose3d.plus(transform);

            // Convert the calculated branch Pose3d to Pose2d
            return branchPose3d.toPose2d().transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI/2)));
        }
        /**
         * Calculates the Pose2d to align to for scoring on L1 near this branch based on the AprilTag's pose and
         * offsets.
         *
         * @return The calculated Pose2d for this reef branch.
         */
        private Pose2d getL1Pose() {
                Pose3d basePose3d = FieldConstants.APRIL_TAGS.get(aprilTagIndex).pose;
                double adjustedYOffset = DriveConstants.ROBOT_WIDTH_WITH_BUMPERS / 2.0;
    
                // Apply both X and Y offsets to calculate the reef branch pose
                Transform3d transform = new Transform3d(adjustedYOffset, -Math.signum(xOffset)*Units.inchesToMeters(37.04/2-1), 0, new Rotation3d(0, 0, 0));
    
                Pose3d branchPose3d = basePose3d.plus(transform);
    
                // Convert the calculated branch Pose3d to Pose2d
                return branchPose3d.toPose2d().transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI/2)));
            }
    
        /**
         * Finds the appropriate reef branch based on the AprilTag ID and whether the
         * pose is left or right.
         *
         * @param aprilTagId The ID of the AprilTag.
         * @param isLeftPose True if the pose is left, false if it's right.
         * @return The matching REEF, or null if no match is found.
         */
        public static REEF fromAprilTagIdAndPose(int aprilTagId, boolean isLeftPose) {
            for (REEF branch : values()) {
                if (!branch.isAlgae && branch.aprilTagId == aprilTagId &&
                        ((isLeftPose && branch.xOffset > 0) || (!isLeftPose && branch.xOffset < 0))) {
                    return branch;
                }
            }
            return null;
        }

        /**
         * Finds the appropriate reef algae position based on the AprilTag ID
         *
         * @param aprilTagId The ID of the AprilTag.
         * @return The matching REEF, or null if no match is found.
         */
        public static REEF fromAprilTagIdAlgae(int aprilTagId) {
            for (REEF branch : values()) {
                if (branch.isAlgae && branch.aprilTagId == aprilTagId) {
                    return branch;
                }
            }
            return null;
        }
    }
}
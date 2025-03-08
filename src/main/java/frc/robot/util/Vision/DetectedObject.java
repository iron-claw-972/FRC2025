package frc.robot.util.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Stores information about an object detected by vision
 */
public class DetectedObject {
    private static Drivetrain drive;
    public final Pose3d pose;
    public final ObjectType type;

    public enum ObjectType{
        CORAL(Units.inchesToMeters(4.5/2)),
        ALGAE(Units.inchesToMeters(16.25/2)),
        RED_ROBOT(0),
        BLUE_ROBOT(0),
        NONE(0);

        public final double height;

        private ObjectType(double h){
            height = h;
        }
    };

    /**
     * Sets the drivetrain to use for pose calculations
     * @param drive The drivetrain
     */
    public static void setDrive(Drivetrain drive){
        DetectedObject.drive = drive;
    }

    /**
     * Creates a default DetectedObject with default attributes
     */
    public DetectedObject(){
        pose = new Pose3d();
        type = ObjectType.NONE;
    }

    /**
     * Creates a new DetectedObject
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param distance The distance from the camera to the object in meters
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     */
    public DetectedObject(double xOffset, double yOffset, double distance, ObjectType type, Transform3d robotToCamera){
        this(xOffset, yOffset, distance, type, robotToCamera, -1);
    }

    /**
     * Creates a new DetectedObject
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param distance The distance from the camera to the object in meters
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     * @param timestamp The timestamp of the picture in seconds
     */
    public DetectedObject(double xOffset, double yOffset, double distance, ObjectType type, Transform3d robotToCamera, double timestamp){
        this.type = type;
        // Get the position relative to the camera
        Translation3d translation = new Translation3d(distance, new Rotation3d(0, -yOffset, -xOffset))
        // Rotate and translate it to get the position relative to the robot
            .rotateBy(robotToCamera.getRotation())
            .plus(robotToCamera.getTranslation());
        // If the drivetrain exists, rotate and translate it to get the field relative position
        if(drive != null){
            Pose2d drivePose = drive.getPoseAt(timestamp);
            translation = translation.rotateBy(new Rotation3d(
                0,
                0,
                drivePose.getRotation().getRadians()
            )).plus(new Translation3d(
                drivePose.getX(),
                drivePose.getY(),
                0
            ));
        }
        pose = new Pose3d(translation, new Rotation3d());
    }
    
    /**
     * Creates a new DetectedObject
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param distance The distance from the camera to the object in meters
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     * @param timestamp The timestamp of the picture in seconds
     */
    public DetectedObject(double xOffset, double yOffset, double distance, int type, Transform3d robotToCamera, double timestamp){
        this(xOffset, yOffset, distance, getType(type), robotToCamera, timestamp);
    }

    /**
     * Creates a new DetectedObject
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param distance The distance from the camera to the object in meters
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     */
    public DetectedObject(double xOffset, double yOffset, double distance, int type, Transform3d robotToCamera){
        this(xOffset, yOffset, distance, getType(type), robotToCamera, -1);
    }

    /**
     * Creates a new DetectedObject
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param distance The distance from the camera to the object in meters
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     * @param timestamp The timestamp of the picture in seconds
     */
    public DetectedObject(double xOffset, double yOffset, double distance, String type, Transform3d robotToCamera, double timestamp){
        this(xOffset, yOffset, distance, getType(type), robotToCamera, timestamp);
    }
    
    /**
     * Creates a new DetectedObject
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param distance The distance from the camera to the object in meters
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     */
    public DetectedObject(double xOffset, double yOffset, double distance, String type, Transform3d robotToCamera){
        this(xOffset, yOffset, distance, getType(type), robotToCamera, -1);
    }

    /**
     * Creates a new DetectedObject, assuming the object is on the ground
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     */
    public DetectedObject(double xOffset, double yOffset, ObjectType type, Transform3d robotToCamera){
        this(xOffset, yOffset, type, robotToCamera, -1);
    }
    
    /**
     * Creates a new DetectedObject, assuming the object is on the ground
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     * @param timestamp The timestamp of the picture in seconds
     */
    public DetectedObject(double xOffset, double yOffset, ObjectType type, Transform3d robotToCamera, double timestamp){
        this.type = type;
        // Get the position relative to the camera
        Translation3d translation = new Translation3d(1, new Rotation3d(0, -yOffset, -xOffset))
        // Rotate it to get the position relative to the rotated camera
            .rotateBy(robotToCamera.getRotation());
        // Scale it so that the object will be on the ground (- because translation's z will be negative)
        if(!isRobot()){
            translation = translation.times(-(robotToCamera.getZ()-type.height)/translation.getZ());
        }else{
            // Assume all robots are ~3m from the camera
            translation = translation.times(3);
        }
        // Translate it to make it relative to the robot
        translation = translation.plus(robotToCamera.getTranslation());
        // If the drivetrain exists, rotate and translate it to be field relative
        if(drive != null){
            Pose2d drivePose = drive.getPoseAt(timestamp);
            translation = translation.rotateBy(new Rotation3d(
                0,
                0,
                drivePose.getRotation().getRadians()
            )).plus(new Translation3d(
                drivePose.getX(),
                drivePose.getY(),
                0
            ));
        }
        pose = new Pose3d(translation, new Rotation3d());
    }
    
    /**
     * Creates a new DetectedObject, assuming the object is on the ground
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     * @param timestamp The timestamp of the picture in seconds
     */
    public DetectedObject(double xOffset, double yOffset, int type, Transform3d robotToCamera, double timestamp){
        this(xOffset, yOffset, getType(type), robotToCamera, timestamp);
    }

    /**
     * Creates a new DetectedObject, assuming the object is on the ground
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     */
    public DetectedObject(double xOffset, double yOffset, int type, Transform3d robotToCamera){
        this(xOffset, yOffset, getType(type), robotToCamera, -1);
    }

    /**
     * Creates a new DetectedObject, assuming the object is on the ground
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     * @param timestamp The timestamp of the picture in seconds
     */
    public DetectedObject(double xOffset, double yOffset, String type, Transform3d robotToCamera, double timestamp){
        this(xOffset, yOffset, getType(type), robotToCamera, timestamp);
    }

    /**
     * Creates a new DetectedObject, assuming the object is on the ground
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     */
    public DetectedObject(double xOffset, double yOffset, String type, Transform3d robotToCamera){
        this(xOffset, yOffset, getType(type), robotToCamera, -1);
    }

    /**
     * Converts an int to an ObjectType
     * @param type The type as an int, between 0 and the number of object types - 1
     * @return The type as an ObjectType
     */
    public static ObjectType getType(int type){
        ObjectType[] values = ObjectType.values();
        if(type < 0 || type >= values.length){
            return ObjectType.NONE;
        }
        return values[type];
    }

    /**
     * Converts a String to an ObjectType
     * @param type The type as a String
     * @return The type as an ObjectType
     */
    public static ObjectType getType(String type){
        ObjectType result = ObjectType.valueOf(type.toUpperCase());
        return result==null ? ObjectType.NONE : result;
    }

    /**
     * Returns if the object is a game piece
     * @return True if the object is a game piece, false otherwise
     */
    public boolean isGamePiece(){
        return type==ObjectType.CORAL || type==ObjectType.ALGAE;
    }
    /**
     * Returns if the object is a robot
     * @return True if the object is a red or blue robot, false otherwise
     */
    public boolean isRobot(){
        return type==ObjectType.RED_ROBOT || type==ObjectType.BLUE_ROBOT;
    }
    /**
     * Returns if the object is a robot on the same alliance
     * @return If the object is a robot on the same alliance
     */
    public boolean isSameAllianceRobot(){
        return type == (Robot.getAlliance()==Alliance.Red?ObjectType.RED_ROBOT:ObjectType.BLUE_ROBOT);
    }
    /**
     * Returns if the object is a robot on the other alliance
     * @return If the object is a robot on the other alliance
     */
    public boolean isOtherAllianceRobot(){
        return type == (Robot.getAlliance()==Alliance.Red?ObjectType.BLUE_ROBOT:ObjectType.RED_ROBOT);
    }

    /**
     * Gets the distance from the center of the robot to the object
     * @return The distance in meters
     */
    public double getDistance(){
        return drive.getPose().getTranslation().getDistance(pose.getTranslation().toTranslation2d());
    }

    /**
     * Gets the field relative angle from the robot to the object
     * @return The angle in radians
     */
    public double getAngle(){
        Pose2d drivePose = drive.getPose();
        return Math.atan2(pose.getY()-drivePose.getY(), pose.getX()-drivePose.getX());
    }

    /**
     * Gets the angle relative to the front of the robot (0 is in front, positive counterclockwise)
     * @return The relative angle in radians
     */
    public double getRelativeAngle(){
        double angle = getAngle()-drive.getYaw().getRadians();
        return MathUtil.angleModulus(angle);
    }

    /**
     * Gets the angle of the object relative to the robot's velocity (0 is in front, positive counterclockwise)
     * @return The relative angle in radians
     */
    public double getVelocityRelativeAngle(){
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        double angle = getRelativeAngle() - Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
        return MathUtil.angleModulus(angle);
    }

    public String toString(){
        return type+" at ("+pose.getX()+", "+pose.getY()+", "+pose.getZ()+")";
    }
}

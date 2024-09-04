package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/**
 * Stores information about an object detected by vision
 */
public class DetectedObject {
    private static Drivetrain drive;
    public final Pose3d pose;
    public final ObjectType type;
    public enum ObjectType{NOTE, RED_ROBOT, BLUE_ROBOT, NONE};

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
        this.type = type;
        // Get the position relative to the camera
        Translation3d translation = new Translation3d(distance, new Rotation3d(0, yOffset, -xOffset));
        // Rotate and translate it to get the position relative to the robot
        translation = translation.rotateBy(robotToCamera.getRotation());
        translation = translation.plus(robotToCamera.getTranslation());
        // If the drivetrain exists, rotate and translate it to get the field relative position
        if(drive != null){
            translation = translation.rotateBy(new Rotation3d(
                0,
                0,
                drive.getYaw().getRadians()
            ));
            Translation2d drivePose = drive.getPose().getTranslation();
            translation = translation.plus(new Translation3d(
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
     */
    public DetectedObject(double xOffset, double yOffset, double distance, long type, Transform3d robotToCamera){
        this(xOffset, yOffset, distance, getType(type), robotToCamera);
    }
    /**
     * Creates a new DetectedObject, assuming the object is on the ground
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     */
    public DetectedObject(double xOffset, double yOffset, ObjectType type, Transform3d robotToCamera){
        this.type = type;
        // Get the position relative to the camera
        Translation3d translation = new Translation3d(1, new Rotation3d(0, yOffset, -xOffset));
        // Rotate it to get the position relative to the rotated camera
        translation = translation.rotateBy(robotToCamera.getRotation());
        // Scale it so that the object will be on the ground (- because translation's z will be negative)
        if(!isRobot()){
            translation = translation.times(-robotToCamera.getZ()/translation.getZ());
        }else{
            // Assume all robots are ~3m from the camera
            translation = translation.times(3);
        }
        // Translate it to make it relative to the robot
        translation = translation.plus(robotToCamera.getTranslation());
        // If the drivetrain exists, rotate and translate it to be field relative
        if(drive != null){
            translation = translation.rotateBy(new Rotation3d(
                0,
                0,
                drive.getYaw().getRadians()
            ));
            Translation2d drivePose = drive.getPose().getTranslation();
            translation = translation.plus(new Translation3d(
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
     */
    public DetectedObject(double xOffset, double yOffset, long type, Transform3d robotToCamera){
        this(xOffset, yOffset, getType(type), robotToCamera);
    }

    /**
     * Converts a String to an ObjectType
     * @param type The type as a String
     * @return The type as an ObjectType
     */
    public static ObjectType getType(long type){
        return
            type==0?ObjectType.NOTE:
            type==1?ObjectType.RED_ROBOT:
            type==2?ObjectType.BLUE_ROBOT:
            ObjectType.NONE;
    }

    /**
     * Returns if the object is a game piece
     * @return True if the object is a note, false otherwise
     */
    public boolean isGamePiece(){
        return type==ObjectType.NOTE;
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
        return Math.atan2(pose.getY()-drive.getPose().getY(), pose.getX()-drive.getPose().getX());
    }

    /**
     * Gets the angle relative to the robot (0 is in front, positive counterclockwise)
     * @return The relative angle in radians
     */
    public double getRelativeAngle(){
        double angle = getAngle()-drive.getYaw().getRadians();
        if(angle > Math.PI){
            angle -= Math.PI*2;
        }else if(angle < -Math.PI){
            angle += Math.PI*2;
        }
        return angle;
    }

    public String toString(){
        return type+" at ("+pose.getX()+", "+pose.getY()+", "+pose.getZ()+")";
    }
}

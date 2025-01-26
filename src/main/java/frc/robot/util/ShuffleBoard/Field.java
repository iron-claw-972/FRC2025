// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DetectedObject;
import frc.robot.util.SwerveModulePose;
import frc.robot.util.Vision;

/** 
 * Displays objects on a Field2d.
 */
public class Field {
    private Drivetrain drive;
    private Vision vision;
    private Field2d field = new Field2d();
    private Pose2d chassisPose = new Pose2d();
    private Pose2d[] aprilTagPoses;
    private SwerveModulePose modulePoses;
    
    public Field(Drivetrain drive, Vision vision){
        this.drive = drive;
        this.vision = vision;
        Shuffleboard.getTab("Swerve").add(field);
        aprilTagPoses = getTagPoses();
        modulePoses = drive.getSwerveModulePose();
    } 
    
    public void updateModulePositions(){
        if (drive.getPose() != null){
            chassisPose = drive.getPose();
        }
    }

    public Pose2d[] getTagPoses(){
        Pose2d[] poses = new Pose2d[FieldConstants.APRIL_TAGS.size()];
        for(int i = 0; i < FieldConstants.APRIL_TAGS.size(); i++){
            poses[i] = FieldConstants.APRIL_TAGS.get(i).pose.toPose2d();
        }
        return poses;
    }

    public void updateFeild(){
        updateModulePositions();
        field.setRobotPose(chassisPose);
        field.getObject("Modules").setPoses(modulePoses.getModulePoses());
        field.getObject("AprilTags").setPoses(aprilTagPoses);
        // For testing, TODO: remove before competition
        field.getObject("Detected objects").setPoses(getObjectPoses());
    }

    private Pose2d[] getObjectPoses(){
        if(vision == null){
            return new Pose2d[0];
        }
        DetectedObject[] objects = vision.getDetectedObjects();
        Pose2d[] result = new Pose2d[objects.length];
        for(int i = 0; i < objects.length; i++){
            result[i] = objects[i].pose.toPose2d();
        }
        return result;
    }
}

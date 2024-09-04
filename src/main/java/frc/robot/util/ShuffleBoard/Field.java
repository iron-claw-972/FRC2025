// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.constants.miscConstants.FieldConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class Field {
    private Drivetrain drive;
    private Field2d field = new Field2d();
    private Pose2d chassisPose = new Pose2d();
    private Pose2d[] modulePositions = new Pose2d[4];
    private Pose2d[] aprilTagPoses;
    
    public Field(Drivetrain drive){
        this.drive = drive;
        Shuffleboard.getTab("Swerve").add(field);
        aprilTagPoses = getTagPoses();
    } 
    
    public void updateModulePositions(){
        
        if (drive.getPose() != null){
            chassisPose = drive.getPose();
        }

        for (int i = 0; i<4; i++ ){
            Translation2d postion = DriveConstants.swerveModuleLocations[i].
            rotateBy(chassisPose.getRotation())
            .plus(chassisPose.getTranslation());

            Rotation2d moduleRotation = drive.getModulePositions()[i].angle
            .plus(chassisPose.getRotation());

            if (drive.getModules()[i].getState().speedMetersPerSecond<0){
                moduleRotation = moduleRotation.plus(Rotation2d.fromDegrees(180));
            }else if(drive.getModules()[i].getState().speedMetersPerSecond == 0 && modulePositions[i] != null){
                // Use previous rotation if it isn't moving
                moduleRotation = modulePositions[i].getRotation();
            }

            modulePositions[i] = new Pose2d(postion, moduleRotation);
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
        field.getObject("modules").setPoses(modulePositions);
        field.getObject("april tags").setPoses(aprilTagPoses);
    }

}

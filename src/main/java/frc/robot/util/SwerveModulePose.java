// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;

/** 
 * Stores and updates the position of each module
 */
public class SwerveModulePose {

    double[] dist = {0,0,0,0};
    double[] angles;
    
    Pose2d[] modulePositions;
    Drivetrain drive;
    Rotation2d prev_heading;

    Translation2d[] moduleTranslation2ds;

    public SwerveModulePose(Drivetrain drive, Translation2d... modulePositions){
        this.drive = drive;
        Pose2d chassiPose2d = drive.getPose();
        prev_heading = drive.getYaw();
        this.moduleTranslation2ds = modulePositions;
        angles = new double[4];
        this.modulePositions = new Pose2d[4];
        for (int i = 0; i<4; i++){
            this.modulePositions[i] = new Pose2d(new Translation2d().
            plus(chassiPose2d.getTranslation()).rotateBy(chassiPose2d.getRotation()), new Rotation2d());
            angles[i] = drive.getModuleStates()[i].angle.getRadians();
        }
    }

    public void update(){
        
        Rotation2d dtheta = drive.getYaw().minus(prev_heading);
        SwerveModuleState[] states = drive.getModuleStates();
        for(int i = 0; i<4; i++){
            double dD = drive.getModules()[i].getPosition().distanceMeters - dist[i];
            double dx = drive.getModules()[i].getAngle().getCos() * dD;
            double dy = drive.getModules()[i].getAngle().getSin() * dD;
            
            Twist2d dPose = new Twist2d(dx, dy, dtheta.getRadians());
            
            
            modulePositions[i] = modulePositions[i].exp(dPose);
            
            dist[i] = drive.getModules()[i].getPosition().distanceMeters;
            angles[i] = drive.getModuleStates()[i].angle.getRadians();
        }
        prev_heading = drive.getYaw();
    }

    public Pose2d[] getModulePoses(){
        return modulePositions;
    }

    public void reset(){
        Pose2d chassisPose2d = drive.getPose();
       // SwerveModuleState[] states = drive.getModuleStates();
        for (int i = 0; i<4; i++){
            this.modulePositions[i] = new Pose2d(moduleTranslation2ds[i].rotateBy(chassisPose2d.getRotation()).plus(chassisPose2d.getTranslation()), new Rotation2d().plus(chassisPose2d.getRotation()));
        }
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class SwerveModulePose {

    double[] dist = {0,0,0,0};
    Translation2d[] moduleTranslations;
    Pose2d[] modulePositions;
    Drivetrain drive;
    Rotation2d prev_heading;

    public SwerveModulePose(Drivetrain drive, Translation2d... modulePositions){
        this.drive = drive;
        Pose2d chassiPose2d = drive.getPose();
        prev_heading = drive.getYaw();
        moduleTranslations = new Translation2d[4];
        this.modulePositions = new Pose2d[4];
        for (int i = 0; i<4; i++){
            moduleTranslations[i] = modulePositions[i].
            plus(chassiPose2d.getTranslation()).rotateBy(chassiPose2d.getRotation());
        }
    }

    public void update(){

        for(int i = 0; i<4; i++){
            double dD = drive.getModules()[i].getPosition().distanceMeters - dist[i];
            double x = drive.getModules()[i].getAngle().getCos() * dD;
            double y = drive.getModules()[i].getAngle().getSin() * dD;
           Translation2d dPose = new Translation2d(x, y).rotateBy(drive.getYaw());
            dist[i] = drive.getModules()[i].getPosition().distanceMeters;
            moduleTranslations[i] = moduleTranslations[i].plus(dPose);
            
            Rotation2d moduleRotation = drive.getModulePositions()[i].angle
            .plus(drive.getYaw());

            if (drive.getModules()[i].getState().speedMetersPerSecond<0){
                moduleRotation = moduleRotation.plus(Rotation2d.fromDegrees(180));
            }else if(drive.getModules()[i].getState().speedMetersPerSecond == 0 && modulePositions[i] != null){
                // Use previous rotation if it isn't moving
                moduleRotation = modulePositions[i].getRotation();
            }

            modulePositions[i] = new Pose2d(moduleTranslations[i], moduleRotation);
        }
        prev_heading = drive.getYaw();
    }

    public Pose2d[] getModuTranslation2ds(){
        return modulePositions;
    }
}

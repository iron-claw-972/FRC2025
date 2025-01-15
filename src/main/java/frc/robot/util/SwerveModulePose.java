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

    private Translation2d[] moduleTranslations;
    private Pose2d[] modulePositions;
    private double[] angles;
    private Drivetrain drive;

    public SwerveModulePose(Drivetrain drive, Translation2d... modulePositions){
        this.drive = drive;
        this.moduleTranslations = modulePositions;
        this.modulePositions = new Pose2d[4];
        angles = new double[4];
        reset();
    }

    public void update(){
        SwerveModuleState[] states = drive.getModuleStates();

        for(int i = 0; i<4; i++){
            Twist2d twist = new Twist2d(states[i].speedMetersPerSecond*Constants.LOOP_TIME, 0, MathUtil.angleModulus(states[i].angle.getRadians()-angles[i]) + drive.getChassisSpeeds().omegaRadiansPerSecond*Constants.LOOP_TIME);
            angles[i] = states[i].angle.getRadians();
            modulePositions[i] = modulePositions[i].exp(twist);
        }
    }

    public Pose2d[] getModulePoses(){
        return modulePositions;
    }

    public void reset(){
        Pose2d chassisPose2d = drive.getPose();
        SwerveModuleState[] states = drive.getModuleStates();
        for (int i = 0; i<4; i++){
            angles[i] = states[i].angle.getRadians();
            this.modulePositions[i] = new Pose2d(moduleTranslations[i].rotateBy(chassisPose2d.getRotation()).plus(chassisPose2d.getTranslation()), new Rotation2d(angles[i]).plus(chassisPose2d.getRotation()));
        }
    }
}

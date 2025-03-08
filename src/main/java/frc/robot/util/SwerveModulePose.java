// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drivetrain.Drivetrain;

/** 
 * Stores and updates the position of each module
 */
public class SwerveModulePose {

    private double[] dist = {0,0,0,0};
    private Translation2d[] moduleTranslations;
    private Pose2d[] modulePositions;
    private double[] angles;
    private Drivetrain drive;
    private double prevRotation;
    private Pose2d[] displayPoses;

    /**
     * Creates a new SwerveModulePose object to store and update the positions of each module
     * @param drive The drivetrain
     * @param modulePositions The translations of the modules relative to the center of the robot
     */
    public SwerveModulePose(Drivetrain drive, Translation2d... modulePositions){
        this.drive = drive;
        this.moduleTranslations = modulePositions;
        this.modulePositions = new Pose2d[4];
        angles = new double[4];
        reset();
        update();
        reset();
    }

    /**
     * Updates the module positions
     */
    public void update(){
        SwerveModuleState[] states = drive.getModuleStates();
        double currentRotation = drive.getYaw().getRadians();
        double chassisRotation = currentRotation - prevRotation;

        for(int i = 0; i<4; i++){
            double position = drive.getModules()[i].getPosition().distanceMeters;
            double distance = position - dist[i];
            dist[i] = position;
            
            Twist2d twist = new Twist2d(distance, 0, MathUtil.angleModulus(states[i].angle.getRadians()-angles[i] + chassisRotation));
            angles[i] = states[i].angle.getRadians();
            modulePositions[i] = modulePositions[i].exp(twist);

            displayPoses[i] = new Pose2d(
                modulePositions[i].getTranslation(),
                EqualsUtil.epsilonEquals(states[i].speedMetersPerSecond, 0, 0.01) ? displayPoses[i].getRotation() :
                    states[i].speedMetersPerSecond < 0 ? modulePositions[i].getRotation().plus(new Rotation2d(Math.PI)) :
                    modulePositions[i].getRotation()
            );
        }
        prevRotation = currentRotation;
    }

    /**
     * Gets the positions of the modules
     * @return The module poses as an array of Pose2ds
     */
    public Pose2d[] getModulePoses(){
        return displayPoses;
    }

    /**
     * Resets the modules to the correct positions relative to the robot
     */
    public void reset(){
        Pose2d chassisPose2d = drive.getPose();
        SwerveModuleState[] states = drive.getModuleStates();
        for (int i = 0; i<4; i++){
            angles[i] = states[i].angle.getRadians();
            this.modulePositions[i] = new Pose2d(moduleTranslations[i].rotateBy(chassisPose2d.getRotation()).plus(chassisPose2d.getTranslation()), new Rotation2d(angles[i]).plus(chassisPose2d.getRotation()));
        }
        prevRotation = drive.getYaw().getRadians();
        displayPoses = Arrays.copyOf(modulePositions, 4);
    }

    /**
     * Gets whehter or not the modules have slipped
     * A module has slipped if it has moved 0.3m (about 1ft) from its correct position relative to the other modules
     * @return True if any of the modules have slipped, false otherwise
     */
    public boolean slipped(){
        Translation2d total = new Translation2d();
        for(Pose2d pose : modulePositions){
            total = total.plus(pose.getTranslation());
        }
        Pose2d drivePose = new Pose2d(total.div(4), drive.getYaw());
        for(int i = 0; i < 4; i++){
            double dist = modulePositions[i].relativeTo(drivePose).getTranslation().getDistance(moduleTranslations[i]);
            if(dist > 0.3){
                return true;
            }
        }
        return false;
    }

    /**
     * Gets the average slip distance
     * @return The average distance between each module and its correct position
     */
    public double getAverageSlip(){
        Translation2d total = new Translation2d();
        for(Pose2d pose : modulePositions){
            total = total.plus(pose.getTranslation());
        }
        Pose2d drivePose = new Pose2d(total.div(4), drive.getYaw());
        double slip = 0;
        for(int i = 0; i < 4; i++){
            slip += modulePositions[i].relativeTo(drivePose).getTranslation().getDistance(moduleTranslations[i]);
        }
        return slip/4;
    }
}
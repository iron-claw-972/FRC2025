// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.SwerveStuff.ModuleLimits;
import frc.robot.util.SwerveStuff.SwerveSetpoint;
import frc.robot.util.SwerveStuff.SwerveSetpointGenerator;

/** Add your docs here. */
public class DriverAssist {

    Drivetrain drive;
    //robot relative
    Pose2d desiredPose = new Pose2d();

    private SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator();

    TrapezoidProfile xProfile = new TrapezoidProfile(new Constraints(5.38, 10.8));
    TrapezoidProfile yProfile = new TrapezoidProfile(new Constraints(5.38, 10.8));
    TrapezoidProfile omegaProfile = new TrapezoidProfile(new Constraints(5.38, 10.8));

    public DriverAssist(Drivetrain drive){
        this.drive = drive;
    }


    public ChassisSpeeds calculate(ChassisSpeeds driverInput){
       SwerveSetpoint nextChassisSpeed = setpointGenerator.generateSetpoint(
            new ModuleLimits(DriveConstants.kMaxSpeed, Double.MAX_VALUE, Double.MAX_VALUE),
            drive.getCurrSetpoint(), ChassisSpeeds.fromFieldRelativeSpeeds(driverInput,drive.getYaw()),
            Constants.LOOP_TIME);

            ChassisSpeeds accel = nextChassisSpeed.chassisSpeeds().minus(drive.getChassisSpeeds()).div(Constants.LOOP_TIME);
            xProfile = new TrapezoidProfile(new Constraints(Double.MAX_VALUE, accel.vxMetersPerSecond));
            yProfile = new TrapezoidProfile(new Constraints(Double.MAX_VALUE, accel.vyMetersPerSecond));
            omegaProfile = new TrapezoidProfile(new Constraints(Double.MAX_VALUE, accel.omegaRadiansPerSecond));

            double xError = xProfile.calculate(Constants.LOOP_TIME,new State(desiredPose.getX(),0),new State(drive.getPose().getX(),drive.getChassisSpeeds().vxMetersPerSecond)).velocity -1; 
        return driverInput;
        
    }

    
    

    public void setDesiredPose(Pose2d dPose2d){
        desiredPose = dPose2d;
    }

}

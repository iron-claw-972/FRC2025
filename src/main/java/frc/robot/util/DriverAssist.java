// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.SwerveStuff.SwerveSetpoint;
import frc.robot.util.SwerveStuff.SwerveSetpointGenerator;

/** 
 * A util class to assist the driver drive to a pose
 */
public class DriverAssist {
    // The amount to correct the driver's inpus by
    // 0 = return unchanged driver inputs, 1 = return a value much closer to the calculated speed, sometimes equal to it
    // This can be greater than 1 to fully correct more of the time, like from farther away
    private static final double CORRECTION_FACTOR = 0.5;

    
    // Variables used for first method
    // The setpoint generator, which limits the acceleration
    private static final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator();
    private static final TrapezoidProfile xProfile = new TrapezoidProfile(new Constraints(DriveConstants.kMaxSpeed, DriveConstants.MAX_LINEAR_ACCEL));
    private static final TrapezoidProfile yProfile = new TrapezoidProfile(new Constraints(DriveConstants.kMaxSpeed, DriveConstants.MAX_LINEAR_ACCEL));
    private static final TrapezoidProfile angleProfile = new TrapezoidProfile(new Constraints(DriveConstants.kMaxAngularSpeed, DriveConstants.MAX_ANGULAR_ACCEL));

    /**
     * Combines the driver input with a speed calculated using a trapezoidal profile
     * @param drive The drivetrain
     * @param driverInput The driver input speed
     * @param desiredPose The pose to drive to
     * @return The new speed
     */
    public static ChassisSpeeds calculate2(Drivetrain drive, ChassisSpeeds driverInput, Pose2d desiredPose) {
        // Do nothing if there is no pose
        if(desiredPose == null){
            return driverInput;
        }

        // Store current states
        Pose2d currentPose = drive.getPose();
        Rotation2d yaw = drive.getYaw();
        ChassisSpeeds driveSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), yaw);
        State xState = new State(currentPose.getX(), driveSpeeds.vxMetersPerSecond);
        State yState = new State(currentPose.getY(), driveSpeeds.vyMetersPerSecond);
        State angleState = new State(currentPose.getRotation().getRadians(), driveSpeeds.omegaRadiansPerSecond);

        // Store goal states
        State xGoal = new State(desiredPose.getX(), 0);
        State yGoal = new State(desiredPose.getY(), 0);
        double rotation = desiredPose.getRotation().getRadians();
        if(rotation - currentPose.getRotation().getRadians() > Math.PI){
            rotation -= 2*Math.PI;
        }else if(rotation - currentPose.getRotation().getRadians() < -Math.PI){
            rotation += 2*Math.PI;
        }
        State angleGoal = new State(rotation, 0);

        // Calculate ideal speeds for next frame
        ChassisSpeeds goal = new ChassisSpeeds(
            xProfile.calculate(Constants.LOOP_TIME, xState, xGoal).velocity,
            yProfile.calculate(Constants.LOOP_TIME, yState, yGoal).velocity,
            angleProfile.calculate(Constants.LOOP_TIME, angleState, angleGoal).velocity
        );

        // This calculates the actual acceleration we can get
        // This is the only thing that needs to be robot relative
        SwerveSetpoint nextSetpoint = setpointGenerator.generateSetpoint(
                DriveConstants.MODULE_LIMITS,
                drive.getCurrSetpoint(), ChassisSpeeds.fromFieldRelativeSpeeds(goal, yaw),
                Constants.LOOP_TIME);
        ChassisSpeeds nextChassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(nextSetpoint.chassisSpeeds(), yaw);

        // This is the speed the driver will be able to get next frame
        // Both speeds need to be obtainable in 1 frame or the driver speed will always be farther away
        SwerveSetpoint driverSetpoint = setpointGenerator.generateSetpoint(
                DriveConstants.MODULE_LIMITS,
                drive.getCurrSetpoint(), ChassisSpeeds.fromFieldRelativeSpeeds(driverInput, yaw),
                Constants.LOOP_TIME);
        ChassisSpeeds driverSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(driverSetpoint.chassisSpeeds(), yaw);

        // The difference between the 2 speeds
        ChassisSpeeds error = nextChassisSpeed.minus(driverSpeeds);
        
        // 1.2*1.2^-distance decreases the amount it correct by as distance increases
        double distanceFactor = 1.2*Math.pow(1.2, -currentPose.getTranslation().getDistance(desiredPose.getTranslation()));

        // Driver input speed
        double driverInputSpeed = Math.hypot(driverInput.vxMetersPerSecond, driverInput.vyMetersPerSecond);

        // The amount to correct by
        ChassisSpeeds correction = error.times(Math.min(CORRECTION_FACTOR * distanceFactor * driverInputSpeed / DriveConstants.kMaxSpeed, 1));
        
        return driverSpeeds.plus(correction);
        // return nextChassisSpeed.times(CORRECTION_FACTOR).plus(driverInput.times(1-CORRECTION_FACTOR));
    }

    // Constants used for second method
    public static final double MAX_VELOCITY_ANGLE_ERROR = Math.PI/4;
    public static final double MAX_DISTANCE_ERROR = 2;
    public static final double ROTATION_CORRECTION_FACTOR = 1;
    public static final double MAX_ROTATION_ERROR = Math.PI/3;

    /**
     * Combines the driver input with a speed perpendicular to the input
     * @param drive The drivetrain
     * @param driverInput The driver input speed
     * @param desiredPose The pose to drive to
     * @return The new speed
     */
    public static ChassisSpeeds calculate(Drivetrain drive, ChassisSpeeds driverInput, Pose2d desiredPose) {
        if(desiredPose == null){
            return driverInput;
        }
        Pose2d drivePose = drive.getPose();
        Translation2d difference = desiredPose.getTranslation().minus(drivePose.getTranslation());
        double distance = difference.getNorm();
        double angleToTarget = difference.getAngle().getRadians();
        double inputSpeed = Math.hypot(driverInput.vxMetersPerSecond, driverInput.vyMetersPerSecond);
        double driverAngle = Math.atan2(driverInput.vyMetersPerSecond, driverInput.vxMetersPerSecond);
        double angleError = MathUtil.angleModulus(angleToTarget - driverAngle);
        if(Math.abs(angleError) > MAX_VELOCITY_ANGLE_ERROR){
            return driverInput;
        }
        double perpendicularDist = Math.abs(distance * Math.sin(angleError));
        if(perpendicularDist > MAX_DISTANCE_ERROR){
            return driverInput;
        }
        double perpendicularAngle = MathUtil.angleModulus(driverAngle + Math.PI/2*Math.signum(angleError));
        // Different options for calculation. From simulator testing, I like the 3rd one the best
        // double correctionSpeed = Math.min(CORRECTION_FACTOR * inputSpeed * Math.pow(2, -perpendicularDist), Math.abs(Math.tan(angleError)*inputSpeed));
        // double correctionSpeed = Math.min(CORRECTION_FACTOR * Math.pow(1.5, -perpendicularDist), 1) * Math.abs(Math.tan(angleError)*inputSpeed);
        double correctionSpeed = Math.min(CORRECTION_FACTOR * inputSpeed * Math.pow(2, -perpendicularDist) * Math.pow(1.2, -distance+1), Math.abs(Math.tan(angleError)*inputSpeed));
        double rotationError = MathUtil.angleModulus(angleToTarget-drivePose.getRotation().getRadians());
        if(Math.abs(rotationError) > MAX_ROTATION_ERROR){
            return driverInput;
        }
        // We want to set the current angular velocity so that we can decelerate to 0rad/s at the setpoint
        // Since 0=v0^2+2ax, v0=√(2ax)
        // High correction factors will also ignore the driver's input more
        double rotationalSpeed = ROTATION_CORRECTION_FACTOR * Math.signum(rotationError)*Math.sqrt(2*DriveConstants.MAX_ANGULAR_ACCEL*Math.abs(rotationError)) - ROTATION_CORRECTION_FACTOR * driverInput.omegaRadiansPerSecond;
        return driverInput.plus(new ChassisSpeeds(correctionSpeed*Math.cos(perpendicularAngle), correctionSpeed*Math.sin(perpendicularAngle), rotationalSpeed));
    }
}
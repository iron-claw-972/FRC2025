// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
 * A util class to assist the driver drive to a pose with 1 method
 */
public class DriverAssist {
    // The amount to correct the driver's inpus by
    // 0 = return unchanged driver inputs, 1 = return calculated speed without driver input
    private static final double CORRECTION_FACTOR = 0.75;

    // The setpoint generator, which limits the acceleration
    private static SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator();

    private static TrapezoidProfile xProfile1 = new TrapezoidProfile(new Constraints(DriveConstants.kMaxSpeed, DriveConstants.MAX_LINEAR_ACCEL));
    private static TrapezoidProfile yProfile1 = new TrapezoidProfile(new Constraints(DriveConstants.kMaxSpeed, DriveConstants.MAX_LINEAR_ACCEL));
    private static TrapezoidProfile angleProfile1 = new TrapezoidProfile(new Constraints(DriveConstants.kMaxAngularSpeed, DriveConstants.MAX_ANGULAR_ACCEL));

    /**
     * Combines the driver input with a speed calculated using a trapezoidal profile
     * @param driverInput The driver input speed
     * @return The new speed
     */
    public static ChassisSpeeds calculate(Drivetrain drive, ChassisSpeeds driverInput, Pose2d desiredPose) {
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
            xProfile1.calculate(Constants.LOOP_TIME, xState, xGoal).velocity,
            yProfile1.calculate(Constants.LOOP_TIME, yState, yGoal).velocity,
            angleProfile1.calculate(Constants.LOOP_TIME, angleState, angleGoal).velocity
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
}

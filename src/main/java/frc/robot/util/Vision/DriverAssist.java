// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.SwerveStuff.SwerveSetpoint;
import frc.robot.util.SwerveStuff.SwerveSetpointGenerator;

/** 
 * A util class to assist the driver drive to a pose
 */
public class DriverAssist {
    // The amount to correct the driver's inpus by
    // 0 = return unchanged driver inputs, 1 = return a value much closer to the calculated speed, sometimes equal to it
    // This can be greater than 1 to fully correct more of the time, like from farther away
    private static final double CORRECTION_FACTOR = 1;

    
    // Variables used for first method
    // The setpoint generator, which limits the acceleration
    private static final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator();
    private static final TrapezoidProfile xProfile = new TrapezoidProfile(new Constraints(DriveConstants.MAX_SPEED, DriveConstants.MAX_LINEAR_ACCEL));
    private static final TrapezoidProfile yProfile = new TrapezoidProfile(new Constraints(DriveConstants.MAX_SPEED, DriveConstants.MAX_LINEAR_ACCEL));
    private static final TrapezoidProfile angleProfile = new TrapezoidProfile(new Constraints(DriveConstants.MAX_ANGULAR_SPEED, DriveConstants.MAX_ANGULAR_ACCEL));

    /**
     * Combines the driver input with a speed calculated using a trapezoidal profile <p>
     * Called when VisionConstants.DRIVER_ASSIST_MODE is 2
     * @param drive The drivetrain
     * @param driverInput The driver input speed
     * @param desiredPose The pose to drive to
     * @param keepAngle True to use the angle in the pose, false to point hte robot toward the pose
     * @return The new speed
     */
    private static ChassisSpeeds calculate2(Drivetrain drive, ChassisSpeeds driverInput, Pose2d desiredPose, boolean keepAngle) {
        // Do nothing if there is no pose
        if(desiredPose == null){
            return driverInput;
        }

        // Store current states
        Pose2d currentPose = drive.getPose();
        Rotation2d yaw = drive.getYaw();
        ChassisSpeeds driveSpeeds = drive.getChassisSpeeds();
        driveSpeeds = ChassisSpeeds. fromFieldRelativeSpeeds(driveSpeeds,yaw); // Changing this does not cause problems because getChassisSpeeds() creates a new object
        State xState = new State(currentPose.getX(), driveSpeeds.vxMetersPerSecond);
        State yState = new State(currentPose.getY(), driveSpeeds.vyMetersPerSecond);
        State angleState = new State(currentPose.getRotation().getRadians(), driveSpeeds.omegaRadiansPerSecond);

        // Store goal states
        State xGoal = new State(desiredPose.getX(), 0);
        State yGoal = new State(desiredPose.getY(), 0);
        Translation2d difference = desiredPose.getTranslation().minus(currentPose.getTranslation());
        double rotation = keepAngle ? desiredPose.getRotation().getRadians() : difference.getAngle().getRadians();
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
        // Robot-relataive goal
        ChassisSpeeds goalRobot = goal.times(1);
        goalRobot = ChassisSpeeds. fromRobotRelativeSpeeds(goalRobot, yaw);

        // This calculates the actual acceleration we can get
        // This is the only thing that needs to be robot relative
        SwerveSetpoint nextSetpoint = setpointGenerator.generateSetpoint(
                DriveConstants.MODULE_LIMITS,
                0,
                drive.getCurrSetpoint(), goalRobot,
                Constants.LOOP_TIME);
            ChassisSpeeds nextChassisSpeed = nextSetpoint.chassisSpeeds();
            nextChassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(nextChassisSpeed, yaw);

        // Robot relative driver inputs
        ChassisSpeeds driverInputRobot = driverInput.times(1); // Copy so original doesn't change
        driverInputRobot = ChassisSpeeds.fromFieldRelativeSpeeds(driverInputRobot, yaw);
        // This is the speed the driver will be able to get next frame
        // Both speeds need to be obtainable in 1 frame or the driver speed will always be farther away
        SwerveSetpoint driverSetpoint = setpointGenerator.generateSetpoint(
                DriveConstants.MODULE_LIMITS,
                0,
                drive.getCurrSetpoint(), driverInputRobot,
                Constants.LOOP_TIME);
        ChassisSpeeds driverSpeeds = driverSetpoint.chassisSpeeds();
        driverSpeeds= ChassisSpeeds.fromRobotRelativeSpeeds(driverSpeeds,yaw);

        // The difference between the 2 speeds
        ChassisSpeeds error = nextChassisSpeed.minus(driverSpeeds);
        
        // 1.2*1.2^-distance decreases the amount it correct by as distance increases
        double distanceFactor = 1.2*Math.pow(1.2, -currentPose.getTranslation().getDistance(desiredPose.getTranslation()));

        // Driver input speed
        double driverInputSpeed = Math.hypot(driverInput.vxMetersPerSecond, driverInput.vyMetersPerSecond);

        // The amount to correct by
        ChassisSpeeds correction = error.times(Math.min(CORRECTION_FACTOR * distanceFactor * driverInputSpeed / DriveConstants.MAX_SPEED, 1));
        
        return driverSpeeds.plus(correction);
        // return nextChassisSpeed.times(CORRECTION_FACTOR).plus(driverInput.times(1-CORRECTION_FACTOR));
    }

    // Constants used for second method
    public static final double MAX_VELOCITY_ANGLE_ERROR = Math.PI/4;
    public static final double MAX_DISTANCE_ERROR = 2;
    public static final double ROTATION_CORRECTION_FACTOR = 0.1;
    public static final double MAX_ROTATION_ERROR = Math.PI/3;

    /**
     * Combines the driver input with a calculated correction speed
     * @param drive The drivetrain
     * @param driverInput The driver input speed
     * @param desiredPose The pose to drive to
     * @param keepAngle True to use the angle in the pose, false to point hte robot toward the pose
     * @return The new speed
     */
    @SuppressWarnings("unused") // Needed because some code might not run for some values of DRIVER_ASSIST_MODE
    public static ChassisSpeeds calculate(Drivetrain drive, ChassisSpeeds driverInput, Pose2d desiredPose, boolean keepAngle) {
        if(VisionConstants.DRIVER_ASSIST_MODE < 2 || desiredPose == null){
            return driverInput;
        }else if(VisionConstants.DRIVER_ASSIST_MODE == 2){
            return calculate2(drive, driverInput, desiredPose, keepAngle);
        }
        // Combines the driver input with a speed perpendicular to the input
        Pose2d drivePose = drive.getPose();
        Translation2d difference = desiredPose.getTranslation().minus(drivePose.getTranslation());
        double distance = difference.getNorm();
        double velocityAngle = difference.getAngle().getRadians();
        double targetAngle = keepAngle ? desiredPose.getRotation().getRadians() : MathUtil.angleModulus(velocityAngle + Math.PI/2);
        double inputSpeed = Math.hypot(driverInput.vxMetersPerSecond, driverInput.vyMetersPerSecond);
        double driverAngle = Math.atan2(driverInput.vyMetersPerSecond, driverInput.vxMetersPerSecond);
        double velocityAngleError = MathUtil.angleModulus(velocityAngle - driverAngle);
        if(Math.abs(velocityAngleError) > MAX_VELOCITY_ANGLE_ERROR){
            return driverInput;
        }
        double perpendicularDist = Math.abs(distance * Math.sin(velocityAngleError));
        if(perpendicularDist > MAX_DISTANCE_ERROR){
            return driverInput;
        }
        double perpendicularAngle = MathUtil.angleModulus(driverAngle + Math.PI/2*Math.signum(velocityAngleError));
        // Different options for calculation.
        double correctionSpeed = 0;
        switch(VisionConstants.DRIVER_ASSIST_MODE){
            case 3:
                correctionSpeed = Math.min(CORRECTION_FACTOR * inputSpeed * Math.pow(2, -perpendicularDist), Math.abs(Math.tan(velocityAngleError)*inputSpeed));
                break;
            case 4:
                correctionSpeed = Math.min(CORRECTION_FACTOR * Math.pow(1.5, -perpendicularDist), 1) * Math.abs(Math.tan(velocityAngleError)*inputSpeed);
                break;
            case 5:
                correctionSpeed = Math.min(CORRECTION_FACTOR * inputSpeed * Math.pow(2, -perpendicularDist) * Math.pow(1.2, -distance+1), Math.abs(Math.tan(velocityAngleError)*inputSpeed));
                break;
        }
        double rotationError = MathUtil.angleModulus(targetAngle-drivePose.getRotation().getRadians());
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

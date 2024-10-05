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

/** Add your docs here. */
public class DriverAssist {
    // The amount to correct the driver's inpus by
    // 0 = return unchanged driver inputs, 1 = return calculated speed without driver input
    private static final double CORRECTION_FACTOR = 0.2;

    // The drivetrain
    private Drivetrain drive;

    // The pose to drive to
    private Pose2d desiredPose;

    // The setpoint generator, which limits the acceleration
    private SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator();

    private TrapezoidProfile xProfile1 = new TrapezoidProfile(new Constraints(DriveConstants.kMaxSpeed, DriveConstants.MAX_LINEAR_ACCEL));
    private TrapezoidProfile yProfile1 = new TrapezoidProfile(new Constraints(DriveConstants.kMaxSpeed, DriveConstants.MAX_LINEAR_ACCEL));
    private TrapezoidProfile angleProfile1 = new TrapezoidProfile(new Constraints(DriveConstants.kMaxAngularSpeed, DriveConstants.MAX_ANGULAR_ACCEL));

    public DriverAssist(Drivetrain drive) {
        this.drive = drive;
    }

    /**
     * Combines the driver input with a speed calculated using a trapezoidal profile
     * @param driverInput The driver input speed
     * @return The new speed
     */
    public ChassisSpeeds calculate(ChassisSpeeds driverInput) {
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

        return nextChassisSpeed.times(CORRECTION_FACTOR).plus(driverInput.times(1-CORRECTION_FACTOR));
    }

    public void setDesiredPose(Pose2d dPose2d) {
        desiredPose = dPose2d;
    }

}

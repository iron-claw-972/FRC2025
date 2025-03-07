package frc.robot.commands.drive_comm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Sets all module angles to a given trajectory's initial angle.
 */
public class TrajectoryPresetSteerAngles extends InstantCommand {
    /*
     * make sure to add wait command after called to give time to correct
     */
    public TrajectoryPresetSteerAngles(Drivetrain drive, Trajectory trajectory) {
        super(
                () -> {

                    // 0.01 is the time between trajectory samples, in seconds
                    // Can be replaced for any small number, but it should be the same as the time between all uses
                    double time = 0.01;

                    drive.setStateDeadband(false);

                    Pose2d initialPose = trajectory.getInitialPose();
                    State sample = trajectory.sample(time);
                    Pose2d nextPose = sample.poseMeters;

                    double xVelocity = sample.velocityMetersPerSecond * nextPose.getRotation().getCos();
                    double yVelocity = sample.velocityMetersPerSecond * nextPose.getRotation().getSin();
                    double angularVelo = (nextPose.getRotation().getRadians() - initialPose.getRotation().getRadians()) / time;

                    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity, angularVelo);
                    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, initialPose.getRotation());

                    SwerveModuleState[] swerveModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
                    for (SwerveModuleState swerveModuleState : swerveModuleStates) {
                        swerveModuleState.speedMetersPerSecond = 0;
                    }
                    drive.setModuleStates(swerveModuleStates, true);
                    drive.setStateDeadband(true);
                },
                drive
             );

    }
}

package frc.robot.commands.drive_comm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Sets the robot's wheels to an X formation to prevent being pushed around by other bots.
 */
public class SetFormationX extends SequentialCommandGroup {
    public SetFormationX(Drivetrain drive) {
        addRequirements(drive);
        addCommands(
            new InstantCommand(() -> drive.setStateDeadband(false), drive),
            new RunCommand(() -> drive.setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45))),
                new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
                new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
                new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45)))
            }, false), drive)
        );
    }
}
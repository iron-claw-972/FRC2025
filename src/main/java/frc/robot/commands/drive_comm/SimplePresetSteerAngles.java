package frc.robot.commands.drive_comm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Attempts to set all four modules to a constant angle. Determines if the modules are able to reach the angle requested in a certain time.
 */
public class SimplePresetSteerAngles extends InstantCommand {

    /**
     * sets the angle of module steer to 0 to remove initial turn time and drift
     *
     * @param drive drivetrain to be used
     */
    public SimplePresetSteerAngles(Drivetrain drive) {
        this(drive, new Rotation2d());
    }

    /**
     * sets the angle of module steer to a angle to remove initial turn time and drift
     *
     * @param angle angle to set module steer to in radians
     * @param drive drivetrain to be used
     */
    public SimplePresetSteerAngles(Drivetrain drive, double angle) {
        this(drive, new Rotation2d(angle));
    }

    /**
     * sets the angle of module steer to a angle to remove initial turn time and drift
     *
     * @param rotation rotation to set module steer to
     * @param drive    drivetrain to be used
     */
    public SimplePresetSteerAngles(Drivetrain drive, Rotation2d rotation) {
        super(() -> {
            drive.setStateDeadband(false);
            drive.setModuleStates(new SwerveModuleState[]{
                    new SwerveModuleState(0, rotation),
                    new SwerveModuleState(0, rotation),
                    new SwerveModuleState(0, rotation),
                    new SwerveModuleState(0, rotation)
            }, true);
            drive.setStateDeadband(true);
        }, drive);
        drive.setStateDeadband(true);
    }
}

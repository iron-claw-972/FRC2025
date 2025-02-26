package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.Drivetrain;


/**
 * Default drive command. Drives robot using driver controls.
 */
public class DefaultDriveCommand extends Command {
    private final Drivetrain swerve;

    public DefaultDriveCommand(
            Drivetrain swerve,
            BaseDriverConfig driver) {
        this.swerve = swerve;
        // anager.logSupplier("DriveControls/ForwardTranslation", () -> driver.getForwardTranslation(), 500, LogLevel.DEBUG);
        // LogManager.logSupplier("DriveControls/SideTranslation", () -> driver.getSideTranslation(), 500, LogLevel.DEBUG);
        // LogMaLogMnager.logSupplier("DriveControls/Rotation", () -> driver.getRotation(), 500, LogLevel.DEBUG);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setStateDeadband(true);
        swerve.enableDriveControls(true);
    }
    

    @Override
    public void end(boolean interrupted) {
        swerve.enableDriveControls(false);
    }
}

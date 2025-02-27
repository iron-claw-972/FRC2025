package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.LogManager;
import frc.robot.util.LogManager.LogLevel;

/**
 * Default drive command. Drives robot using driver controls.
 */
public class DefaultDriveCommand extends Command {
    private final Drivetrain swerve;

    public DefaultDriveCommand(
            Drivetrain swerve,
            BaseDriverConfig driver) {
        this.swerve = swerve;
        LogManager.logSupplier("DriveControls/ForwardTranslation", () -> driver.getForwardTranslation(), 501, LogLevel.DEBUG);
        LogManager.logSupplier("DriveControls/SideTranslation", () -> driver.getSideTranslation(), 503, LogLevel.DEBUG);
        LogManager.logSupplier("DriveControls/Rotation", () -> driver.getRotation(), 509, LogLevel.DEBUG);
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

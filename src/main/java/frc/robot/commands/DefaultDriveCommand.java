package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.LogManager;

/**
 * Default drive command. Drives robot using driver controls.
 */
public class DefaultDriveCommand extends Command {
    private final Drivetrain swerve;
    private final BaseDriverConfig driver;

    public DefaultDriveCommand(
            Drivetrain swerve,
            BaseDriverConfig driver) {
        this.swerve = swerve;
        this.driver = driver;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setStateDeadband(true);
        if (Constants.DO_LOGGING) {
            LogManager.add("DriveControls/ForwardTranslation", () -> driver.getForwardTranslation());
            LogManager.add("DriveControls/SideTranslation", () -> driver.getSideTranslation());
            LogManager.add("DriveControls/Rotation", () -> driver.getRotation());
        }
    }

    @Override
    public void execute() {

        double forwardTranslation = driver.getForwardTranslation();
        double sideTranslation = driver.getSideTranslation();
        double rotation = -driver.getRotation();

        double slowFactor = driver.getIsSlowMode() ? DriveConstants.kSlowDriveFactor : 1;

        forwardTranslation *= slowFactor;
        sideTranslation *= slowFactor;
        rotation *= driver.getIsSlowMode() ? DriveConstants.kSlowRotFactor : 1;

        int allianceReversal = Robot.getAlliance() == Alliance.Red ? 1 : -1;
        forwardTranslation *= allianceReversal;
        sideTranslation *= allianceReversal;

        // If the driver is pressing the align button or a command set the drivetrain to
        // align, then align to speaker
        if (driver.getIsAlign() || swerve.getIsAlign()) {
            swerve.driveHeading(
                    forwardTranslation,
                    sideTranslation,
                    swerve.getAlignAngle(),
                    true);
        } else {
            swerve.drive(
                    forwardTranslation,
                    sideTranslation,
                    rotation,
                    true,
                    false);
        }
    }
}

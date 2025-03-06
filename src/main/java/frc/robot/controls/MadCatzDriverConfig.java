package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import lib.controllers.MadCatzController;
import lib.controllers.MadCatzController.MadCatzAxis;
import lib.controllers.MadCatzController.MadCatzButton;

/**
 * Driver controls for the MadCatz controller.
 */
public class MadCatzDriverConfig extends BaseDriverConfig {

    private final MadCatzController kDriver = new MadCatzController(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = kDriver.get(MadCatzButton.B6);

    public MadCatzDriverConfig(Drivetrain drive) {
        super(drive);
    }

    @Override
    public void configureControls() {
        kDriver.get(MadCatzButton.B1).whileTrue(new SetFormationX(super.getDrivetrain()));
        kDriver.get(MadCatzButton.B2).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(DriveConstants.STARTING_HEADING)));
    }

    @Override
    public double getRawSideTranslation() {
        return kDriver.get(MadCatzAxis.X);
    }

    @Override
    public double getRawForwardTranslation() {
        return -kDriver.get(MadCatzAxis.Y);
    }

    @Override
    public double getRawRotation() {
        return kDriver.get(MadCatzAxis.ZROTATE);
    }

    @Override
    public double getRawHeadingAngle() {
        return kDriver.get(MadCatzAxis.ZROTATE) * Math.PI;
    }

    @Override
    public double getRawHeadingMagnitude() {
        return kDriver.get(MadCatzAxis.SLIDER);
    }

    @Override
    public boolean getIsSlowMode() {
        return slowModeSupplier.getAsBoolean();
    }

    @Override
    public boolean getIsAlign() {
        return false;
    }

}
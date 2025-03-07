package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import lib.controllers.Ex3DProController;
import lib.controllers.Ex3DProController.Ex3DProAxis;
import lib.controllers.Ex3DProController.Ex3DProButton;

/**
 * Driver controls for the Ex3D Pro controller.
 */
public class Ex3DProDriverConfig extends BaseDriverConfig {

    private final Ex3DProController kDriver = new Ex3DProController(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = kDriver.get(Ex3DProButton.B11);

    public Ex3DProDriverConfig(Drivetrain drive) {
        super(drive);
    }

    @Override
    public void configureControls() {
        kDriver.get(Ex3DProButton.B1).whileTrue(new SetFormationX(super.getDrivetrain()));
        kDriver.get(Ex3DProButton.B2).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(DriveConstants.STARTING_HEADING)));
    }

    @Override
    public double getRawSideTranslation() {
        return -kDriver.get(Ex3DProAxis.X);
    }

    @Override
    public double getRawForwardTranslation() {
        return -kDriver.get(Ex3DProAxis.Y);
    }

    @Override
    public double getRawRotation() {
        return kDriver.get(Ex3DProAxis.Z);
    }

    @Override
    public double getRawHeadingAngle() {
        return kDriver.get(Ex3DProAxis.Z) * Math.PI;
    }

    @Override
    public double getRawHeadingMagnitude() {
        return kDriver.get(Ex3DProAxis.SLIDER);
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

package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Robot;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.MathUtils;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {

    private final PS5Controller kDriver = new PS5Controller(Constants.DRIVER_JOY);

    public PS5ControllerDriverConfig(Drivetrain drive) {
        super(drive);
    }

    public void configureControls() {
        // reset the yaw forward. Mainly useful for testing/driver practice
        kDriver.get(PS5Button.OPTIONS).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
                new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI)
                                                                                                   )));

        // reset the yaw backward. Mainly useful for testing/driver practice
        kDriver.get(PS5Button.CREATE).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
                new Rotation2d(Robot.getAlliance() == Alliance.Red ? 0 : Math.PI)
                                                                                                  )));

        // set the wheels to X
        kDriver.get(PS5Button.SQUARE).whileTrue(new RepeatCommand(new SetFormationX(getDrivetrain())));


        // Resets the modules to absolute if they are having the unresolved zeroing error
        kDriver.get(PS5Button.CROSS).onTrue(new InstantCommand(() ->
                getDrivetrain().resetModulesToAbsolute()
        ));

    }


    @Override
    public double getRawSideTranslation() {
        return kDriver.get(PS5Axis.LEFT_X);
    }

    @Override
    public double getRawForwardTranslation() {
        return kDriver.get(PS5Axis.LEFT_Y);
    }

    @Override
    public double getRawRotation() {
        return kDriver.get(PS5Axis.RIGHT_X);
    }

    @Override
    public double getRawHeadingAngle() {
        return Math.atan2(kDriver.get(PS5Axis.RIGHT_X), -kDriver.get(PS5Axis.RIGHT_Y)) - Math.PI / 2;
    }

    @Override
    public double getRawHeadingMagnitude() {
        return MathUtils.calculateHypotenuse(kDriver.get(PS5Axis.RIGHT_X), kDriver.get(PS5Axis.RIGHT_Y));
    }

    @Override
    public boolean getIsSlowMode() {
        return kDriver.get(PS5Button.RIGHT_TRIGGER).getAsBoolean();
    }

    @Override
    public boolean getIsAlign() {
        return kDriver.get(PS5Button.LEFT_TRIGGER).getAsBoolean();
    }
}

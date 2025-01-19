package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.commands.vision.DriverAssistIntake;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Elevator;
import frc.robot.subsystems.gpm.Outtake;
import frc.robot.util.MathUtils;
import frc.robot.util.Vision;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {

    private final PS5Controller kDriver = new PS5Controller(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = kDriver.get(PS5Button.RIGHT_TRIGGER);

    private final Vision vision;
    private final Elevator elevator;
    private final Outtake outtake;
    private final Trigger slowModeTrigger = kDriver.get(PS5Button.RIGHT_TRIGGER);

    public PS5ControllerDriverConfig(Drivetrain drive, Vision vision, Elevator elevator, Outtake outtake) {
        super(drive);
        this.vision = vision;
        this.elevator = elevator;
        this.outtake = outtake;
    }

    @SuppressWarnings("unused")
    public void configureControls() {
        // Reset yaw to be away from driver
        kDriver.get(PS5Button.OPTIONS).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
            new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI))));
        if (elevator!=null && outtake!=null){
            //set the wheels to X
            kDriver.get(PS5Button.CIRCLE).whileTrue(new SetFormationX(super.getDrivetrain()));
            //Enable state deadband after setting formation to X
            kDriver.get(PS5Button.CIRCLE).onFalse(new InstantCommand(()->getDrivetrain().setStateDeadband(true)));

            // Resets the modules to absolute if they are having the unresolved zeroing
            // error
            kDriver.get(PS5Button.CREATE).onTrue(new InstantCommand(() -> getDrivetrain().resetModulesToAbsolute()));

            kDriver.get(PS5Button.TRIANGLE).onTrue(new MoveElevator(elevator, ElevatorConstants.STOW_SETPOINT));
            kDriver.get(PS5Button.LB).onTrue(new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT));
            kDriver.get(PS5Button.RB).onTrue(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT));
            kDriver.get(PS5Button.LEFT_TRIGGER).onTrue(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT));
            kDriver.get(PS5Button.SQUARE).onTrue(new MoveElevator(elevator, ElevatorConstants.INTAKE_SETPOINT));
            kDriver.get(PS5Button.CROSS).onTrue(new OuttakeCoral(outtake, elevator));

            if(vision != null && VisionConstants.DRIVER_ASSIST_MODE > 0){
                // This will only be true when it is equal to 1, but <=1 avoids a warning for comparing identical expressions
                if(VisionConstants.DRIVER_ASSIST_MODE <= 1){
                    kDriver.get(PS5Button.LEFT_TRIGGER).whileTrue(new DriverAssistIntake(getDrivetrain(), this, vision));
                }else{
                    kDriver.get(PS5Button.LEFT_TRIGGER)
                    .onTrue(new InstantCommand(()->getDrivetrain().setDesiredPose(()->vision.getBestGamePiece(Units.degreesToRadians(60), false).pose.toPose2d())))
                    .onFalse(new InstantCommand(()->getDrivetrain().setDesiredPose(()->null)));
                }
            }
        }
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
        return Math.hypot(kDriver.get(PS5Axis.RIGHT_X), kDriver.get(PS5Axis.RIGHT_Y));
    }

    @Override
    public boolean getIsSlowMode() {
        return slowModeTrigger.getAsBoolean();
    }

    @Override
    public boolean getIsAlign() {
        return false;
    }
}

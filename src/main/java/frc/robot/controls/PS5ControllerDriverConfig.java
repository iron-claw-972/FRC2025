package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.commands.gpm.IntakeAlgae;
import frc.robot.commands.gpm.IntakeCoral;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.OuttakeAlgae;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.commands.gpm.ReverseMotors;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.util.MathUtils;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.DPad;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {

    private final PS5Controller driver = new PS5Controller(Constants.DRIVER_JOY);
    private final Elevator elevator;
    private final Intake intake;
    private final Outtake outtake;
    private final Climb climb;
    private final AlgaeIntake algaeIntake;
    private final BooleanSupplier slowModeSupplier = driver.get(PS5Button.RIGHT_TRIGGER);
    private int alignmentDirection = -1;

    public PS5ControllerDriverConfig(Drivetrain drive, Elevator elevator, Intake intake, Outtake outtake, Climb climb, AlgaeIntake algaeIntake) {
        super(drive);
        this.elevator = elevator;
        this.intake = intake;
        this.outtake = outtake;
        this.climb = climb;
        this.algaeIntake = algaeIntake;
    }

    public void configureControls() {
        Trigger menu = driver.get(PS5Button.LEFT_JOY);

        // Elevator setpoints
        driver.get(PS5Button.CREATE).onTrue(new MoveElevator(elevator, ElevatorConstants.L1_SETPOINT));
        driver.get(PS5Button.LB).onTrue(new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT));
        driver.get(PS5Button.RB).and(menu.negate()).onTrue(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT));
        driver.get(PS5Button.LEFT_TRIGGER).onTrue(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT));
        driver.get(PS5Button.TRIANGLE).and(menu.negate()).onTrue(new MoveElevator(elevator, ElevatorConstants.STOW_SETPOINT));

        // Intake/outtake
        driver.get(PS5Button.CROSS).and(menu.negate()).whileTrue(new IntakeCoral(intake, elevator));
        driver.get(PS5Button.PS).and(menu.negate()).onTrue(new OuttakeCoral(outtake, elevator));
        driver.get(PS5Button.CROSS).and(menu).whileTrue(new IntakeAlgae(algaeIntake));
        driver.get(PS5Button.PS).and(menu).onTrue(new OuttakeAlgae(algaeIntake));
        driver.get(PS5Button.CIRCLE).and(menu.negate()).whileTrue(new ReverseMotors(intake, outtake));

        // Climb
        driver.get(PS5Button.SQUARE).and(menu.negate()).onTrue(new InstantCommand(()->climb.extend(), climb))
            .onFalse(new InstantCommand(()->climb.climb(), climb));

        // Alignment
        driver.get(PS5Button.CIRCLE).and(menu).onTrue(new InstantCommand(()->alignmentDirection = 0));
        driver.get(PS5Button.RB).and(menu).onTrue(new InstantCommand(()->alignmentDirection = 1));
        driver.get(PS5Button.TRIANGLE).and(menu).onTrue(new InstantCommand(()->alignmentDirection = 2));
        driver.get(PS5Button.SQUARE).and(menu).onTrue(new InstantCommand(()->alignmentDirection = 3));
        driver.get(DPad.DOWN).onTrue(new InstantCommand(()->alignmentDirection = 4));
        driver.get(DPad.UP).onTrue(new InstantCommand(()->alignmentDirection = 5));
        driver.get(DPad.LEFT).onTrue(new InstantCommand(()->setAlignmentPose(true)));
        driver.get(DPad.RIGHT).onTrue(new InstantCommand(()->setAlignmentPose(false)));

        // Reset the yaw. Mainly useful for testing/driver practice
        driver.get(PS5Button.OPTIONS).onTrue(new InstantCommand(() -> getDrivetrain().setYaw(
                new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI)
        )));

        // Set the wheels to X
        driver.get(PS5Button.SQUARE).whileTrue(new SetFormationX(getDrivetrain()))
            .onFalse(new InstantCommand(()->getDrivetrain().setStateDeadband(true)));

        // Resets the modules to absolute if they are having the unresolved zeroing error
        driver.get(PS5Button.CROSS).onTrue(new InstantCommand(() ->
                getDrivetrain().resetModulesToAbsolute()
        ));
    }

    private void setAlignmentPose(boolean isLeft){
        Pose2d pose = VisionConstants.REEF.fromAprilTagIdAndPose(
            Robot.getAlliance() == Alliance.Blue ? alignmentDirection + 17
            : (8-alignmentDirection) % 6 + 6,
        isLeft).pose;
        getDrivetrain().setDesiredPose(pose);
    }

    @Override
    public double getRawSideTranslation() {
        return driver.get(PS5Axis.LEFT_X);
    }

    @Override
    public double getRawForwardTranslation() {
        return driver.get(PS5Axis.LEFT_Y);
    }

    @Override
    public double getRawRotation() {
        return driver.get(PS5Axis.RIGHT_X);
    }

    @Override
    public double getRawHeadingAngle() {
        return Math.atan2(driver.get(PS5Axis.RIGHT_X), -driver.get(PS5Axis.RIGHT_Y)) - Math.PI / 2;
    }

    @Override
    public double getRawHeadingMagnitude() {
        return MathUtils.calculateHypotenuse(driver.get(PS5Axis.RIGHT_X), driver.get(PS5Axis.RIGHT_Y));
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

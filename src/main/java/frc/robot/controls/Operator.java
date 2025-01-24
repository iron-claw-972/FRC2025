// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.gpm.IntakeAlgae;
import frc.robot.commands.gpm.IntakeCoral;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.OuttakeAlgae;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.commands.gpm.ReverseMotors;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

/** 
 * Controls for the operator, which are almost a duplicate of most of the driver's controls
 */
public class Operator {

    private final GameController driver = new GameController(Constants.OPERATOR_JOY);

    private final Drivetrain drive;
    private final Elevator elevator;
    private final Intake intake;
    private final Outtake outtake;
    private final Climb climb;
    private int alignmentDirection = -1;
    
    public Operator(Drivetrain drive, Elevator elevator, Intake intake, Outtake outtake, Climb climb) {
        this.drive = drive;
        this.elevator = elevator;
        this.intake = intake;
        this.outtake = outtake;
        this.climb = climb;
    }

    public void configureControls() {
        driver.get(Button.BACK).onTrue(new InstantCommand(()->{
            if(elevator != null){
                elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
            }
            if(climb != null){
                climb.climb();
            }
            CommandScheduler.getInstance().cancelAll();
        }));

        Trigger menu = driver.get(Button.LEFT_JOY);

        // Elevator setpoints
        if(elevator != null){
            driver.get(Button.START).onTrue(new MoveElevator(elevator, ElevatorConstants.L1_SETPOINT));
            driver.get(Button.LB).onTrue(new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT));
            driver.get(Button.RB).and(menu.negate()).onTrue(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT));
            getLeftTrigger().onTrue(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT));
            driver.get(Button.Y).and(menu.negate()).onTrue(new MoveElevator(elevator, ElevatorConstants.STOW_SETPOINT));
        }

        // Intake/outtake
        if(intake != null && elevator != null){
            driver.get(Button.A).and(menu.negate()).whileTrue(new IntakeCoral(intake, elevator));
        }
        if(outtake != null && elevator != null){
            driver.get(Button.RIGHT_JOY).and(menu.negate()).onTrue(new OuttakeCoral(outtake, elevator));
        }
        if(intake != null){
            driver.get(Button.A).and(menu).whileTrue(new IntakeAlgae(intake));
            driver.get(Button.RIGHT_JOY).and(menu).onTrue(new OuttakeAlgae(intake));
        }
        if(intake != null && outtake != null){
            driver.get(Button.B).and(menu.negate()).whileTrue(new ReverseMotors(intake, outtake));
        }

        // Climb
        if(climb != null){
            driver.get(Button.X).and(menu.negate()).onTrue(new InstantCommand(()->climb.extend(), climb))
                .onFalse(new InstantCommand(()->climb.climb(), climb));
        }

        // Alignment
        driver.get(Button.B).and(menu).onTrue(new InstantCommand(()->alignmentDirection = 0));
        driver.get(Button.RB).and(menu).onTrue(new InstantCommand(()->alignmentDirection = 1));
        driver.get(Button.Y).and(menu).onTrue(new InstantCommand(()->alignmentDirection = 2));
        driver.get(Button.X).and(menu).onTrue(new InstantCommand(()->alignmentDirection = 3));
        driver.get(DPad.DOWN).onTrue(new InstantCommand(()->alignmentDirection = 4));
        driver.get(DPad.UP).onTrue(new InstantCommand(()->alignmentDirection = 5));
        driver.get(DPad.LEFT).onTrue(new InstantCommand(()->setAlignmentPose(true)));
        driver.get(DPad.RIGHT).onTrue(new InstantCommand(()->setAlignmentPose(false)));
    }

    /**
     * Sets the drivetrain's alignmetn pose to the selected position
     * @param isLeft True for left branch, false for right
     */
    private void setAlignmentPose(boolean isLeft){
        Pose2d pose = VisionConstants.REEF.fromAprilTagIdAndPose(
            Robot.getAlliance() == Alliance.Blue ? alignmentDirection + 17
            : (8-alignmentDirection) % 6 + 6,
        isLeft).pose;
        drive.setDesiredPose(pose);
    }

    public Pose2d getBranch(int direction, boolean isLeft){
        int redId = (direction+3)%6+6;
        int id = Robot.getAlliance() == Alliance.Blue ? redId + 11: redId;
        return REEF.fromAprilTagIdAndPose(id, isLeft).pose;
    }

    public Trigger getRightTrigger(){
        return new Trigger(driver.RIGHT_TRIGGER_BUTTON);
    }
    public Trigger getLeftTrigger(){
        return new Trigger(driver.LEFT_TRIGGER_BUTTON);
    }
    public GameController getGameController(){
        return driver;
    }
}

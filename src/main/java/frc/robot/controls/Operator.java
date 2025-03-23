// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.gpm.IntakeAlgae;
import frc.robot.commands.gpm.IntakeCoral;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.OuttakeAlgaeIntake;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.commands.gpm.RemoveAlgae;
import frc.robot.commands.gpm.ReverseMotors;
import frc.robot.commands.gpm.StationIntake;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import lib.controllers.GameController;
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
    private final Indexer indexer;
    private final Outtake outtake;
    private final Climb climb;
    private final Arm arm;
    private int alignmentDirection = 0;
    
    public Operator(Drivetrain drive, Elevator elevator, Intake intake, Indexer indexer, Outtake outtake, Climb climb, Arm arm) {
        this.drive = drive;
        this.elevator = elevator;
        this.intake = intake;
        this.indexer = indexer;
        this.outtake = outtake;
        this.climb = climb;
        this.arm = arm;
    }

    public void configureControls() {
        // TODO: Update to match new driver controls, except the ones that move the drivetrain
        Trigger menu = driver.get(Button.LEFT_JOY);

        // Elevator setpoints
        if(elevator != null && outtake != null) {
            driver.get(Button.BACK).onTrue(new MoveElevator(elevator, ElevatorConstants.L1_SETPOINT).deadlineFor(new RemoveAlgae(outtake)));
            driver.get(Button.LB).onTrue(new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT).deadlineFor(new RemoveAlgae(outtake)));
            driver.get(Button.RB).and(menu.negate()).onTrue(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT).deadlineFor(new RemoveAlgae(outtake)));
            driver.get(driver.LEFT_TRIGGER_BUTTON).onTrue(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT).deadlineFor(new RemoveAlgae(outtake)));
            driver.get(Button.Y).and(menu.negate()).onTrue(new MoveElevator(elevator, ElevatorConstants.STOW_SETPOINT));
        }

        // Intake/outtake
        Trigger r3 = driver.get(Button.RIGHT_JOY);
        if(intake != null && indexer != null){// && elevator != null){
            driver.get(Button.A).and(menu.negate()).and(r3.negate()).whileTrue(new IntakeCoral(intake, indexer, elevator, outtake, arm));
            // On true, run the command to start intaking
            // On false, run the command to finish intaking if it has a coral
            Command startIntake = new StationIntake(outtake);
            Command finishIntake = new DoNothing();
            driver.get(Button.A).and(r3).and(menu.negate()).onTrue(startIntake)
                .onFalse(new InstantCommand(()->{
                    if(!startIntake.isScheduled()){
                        finishIntake.schedule();
                    }else{
                        startIntake.cancel();
                    }
            }));
        }
        if(intake != null){
            driver.get(Button.A).and(menu).whileTrue(new IntakeAlgae(intake));
            driver.get(DPad.DOWN).and(menu).onTrue(new OuttakeAlgaeIntake(intake));
        }
        if(outtake != null && elevator != null){
            driver.get(DPad.DOWN).and(menu.negate()).onTrue(new OuttakeCoral(outtake, elevator, arm));
        }
        if(intake != null && indexer != null){
            driver.get(Button.B).and(menu.negate()).whileTrue(new ReverseMotors(intake, indexer, outtake));
        }

        // Climb
        if(climb != null){
            driver.get(Button.X).and(menu.negate()).onTrue(new InstantCommand(()->climb.extend(), climb))
                .onFalse(new InstantCommand(()->climb.climb(), climb));
            if(intake != null){
                driver.get(Button.X).and(menu.negate()).onTrue(new InstantCommand(()->intake.setAngle(IntakeConstants.ALGAE_SETPOINT), intake));
            }
        }

        // Alignment
        driver.get(Button.B).and(menu).onTrue(new InstantCommand(()->alignmentDirection = 0));
        driver.get(Button.Y).and(menu).onTrue(new InstantCommand(()->alignmentDirection = 2));
        driver.get(Button.X).and(menu).onTrue(new InstantCommand(()->alignmentDirection = 3));
        driver.get(Button.RB).onTrue(new InstantCommand(()->alignmentDirection = 4));
        driver.get(DPad.UP).onTrue(new InstantCommand(()->alignmentDirection = 5));
        driver.get(DPad.LEFT).onTrue(new InstantCommand(()->setAlignmentPose(true)));
        driver.get(DPad.RIGHT).onTrue(new InstantCommand(()->setAlignmentPose(false)));

        // Cancel commands
        driver.get(Button.START).onTrue(new InstantCommand(()->{
            if(elevator != null){
                elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
            }
            if(outtake != null){
                outtake.stop();
            }
            if(intake != null){
                intake.stow();
                intake.deactivate();
            }
            if(indexer != null){
                indexer.stop();
            }
            if(climb != null){
                climb.stow();
            }
            drive.setDesiredPose(()->null);
            CommandScheduler.getInstance().cancelAll();
        }));
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

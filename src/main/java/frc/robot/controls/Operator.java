// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Elevator;
import frc.robot.subsystems.gpm.Outtake;
import lib.controllers.GameController;

/** Add your docs here. */
public class Operator {

    private final GameController kDriver = new GameController(Constants.OPERATOR_JOY);

    private Drivetrain drive;
    private Elevator elevator;
    private Outtake outtake;
    
    public Operator(Drivetrain drive, Elevator elevator, Outtake outtake) {
        this.drive = drive;
        this.elevator = elevator;
        this.outtake = outtake;
    }

    public void configureControls() {
        // kDriver.get(Button.BACK).onTrue(new InstantCommand(()->drive.setMaxAccel(1.68213715)).
        // andThen(new InstantCommand(()->{
        //     if(elevator != null){
        //         elevator.setSetpoint(ElevatorConstants.L4_SETPOINT);
        //     }
        //     CommandScheduler.getInstance().cancelAll();
        // })));
    
        // kDriver.get(Button.BACK).onFalse(new InstantCommand(()->{
        //     if(elevator != null){
        //         elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
        //     }
        //     CommandScheduler.getInstance().cancelAll();
        // }));
        // }).andThen(new InstantCommand(()->drive.setMaxAccel(DriveConstants.MAX_LINEAR_ACCEL))));
    }
    public Trigger getRightTrigger(){
        return new Trigger(kDriver.RIGHT_TRIGGER_BUTTON);
    }
    public Trigger getLeftTrigger(){
        return new Trigger(kDriver.LEFT_TRIGGER_BUTTON);
    }
    public GameController getGameController(){
        return kDriver;
    }
}

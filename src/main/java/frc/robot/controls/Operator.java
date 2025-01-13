// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Elevator;
import frc.robot.subsystems.gpm.Outtake;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/** Add your docs here. */
public class Operator {

    private final GameController kDriver = new GameController(Constants.OPERATOR_JOY);

    private Drivetrain drive;
    private Elevator elevator;
    private Outtake outtake;
    private int direction = 0;
    
    public Operator(Drivetrain drive, Elevator elevator, Outtake outtake) {
        this.drive = drive;
        this.elevator = elevator;
        this.outtake = outtake;
    }

    public void configureControls() {
        kDriver.get(Button.BACK).onTrue(new InstantCommand(()->{
            if(elevator != null){
                elevator.setSetpoint(ElevatorConstants.L4_SETPOINT);
            }
            if(outtake != null){
                outtake.stop();
            }
            CommandScheduler.getInstance().cancelAll();
        }));
        new Trigger(()->(Math.hypot(kDriver.get(Axis.LEFT_X), kDriver.get(Axis.LEFT_Y)) > 0.5)).onTrue(new InstantCommand(()->{
            double angle = MathUtil.inputModulus(Math.atan2(kDriver.get(Axis.LEFT_Y), kDriver.get(Axis.LEFT_X)), 0, 2*Math.PI);
            direction = (int)(angle/(Math.PI/3));
        }));
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

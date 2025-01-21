// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GroundIntakePrototype;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

/** Add your docs here. */
public class Operator {

    private final GameController kDriver = new GameController(Constants.OPERATOR_JOY);

    private Drivetrain drive;
    private GroundIntakePrototype groundIntakePrototype;
    
    public Operator(Drivetrain drive, GroundIntakePrototype groundIntakePrototype) {
        this.drive = drive;
        this.groundIntakePrototype = groundIntakePrototype;

    }

    public void configureControls() {
        // (indexMotorSpeed, intakeMotorSpeed)
        // go
        kDriver.get(Button.A).whileTrue(new InstantCommand(()->groundIntakePrototype.setBothMotors(0.1, 0.1)));
        // stop
        kDriver.get(Button.X).whileTrue(new InstantCommand(()->groundIntakePrototype.setBothMotors(0,0))); 
        // reverse
        kDriver.get(Button.Y).whileTrue(new InstantCommand(()->groundIntakePrototype.setBothMotors(-0.1, -0.1)));
        

        kDriver.get(Button.BACK).onTrue(new InstantCommand(()->{
            CommandScheduler.getInstance().cancelAll();
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

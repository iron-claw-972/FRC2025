// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.GoToPose;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants.REEF;
import frc.robot.subsystems.Drivetrain;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/** Add your docs here. */
public class Operator {

    private final GameController kDriver = new GameController(Constants.OPERATOR_JOY);

    private Drivetrain drive;
    private int direction = 0;
    
    public Operator(Drivetrain drive) {
        this.drive = drive;
    }

    public void configureControls() {
        kDriver.get(Button.BACK).onTrue(new InstantCommand(()->{
            CommandScheduler.getInstance().cancelAll();
        }));
        new Trigger(()->(Math.hypot(kDriver.get(Axis.LEFT_X), kDriver.get(Axis.LEFT_Y)) > 0.5)).onTrue(new InstantCommand(()->{
            double angle = MathUtil.inputModulus(Math.atan2(-kDriver.get(Axis.LEFT_Y), 
                (Robot.getAlliance() == Alliance.Blue ? -1 : 1) * kDriver.get(Axis.LEFT_X)), 0, 2*Math.PI);
            direction = (int)(angle/(Math.PI/3));
        }));
        // kDriver.get(Button.LB).onTrue(new InstantCommand(()->drive.setDesiredPose(getBranch(direction, true))));
        // kDriver.get(Button.RB).onTrue(new InstantCommand(()->drive.setDesiredPose(getBranch(direction, false))));
        kDriver.get(Button.LB).whileTrue(new GoToPose(()->getBranch(direction, true), 2, 5, drive));
        kDriver.get(Button.RB).whileTrue(new GoToPose(()->getBranch(direction, false), 2, 5, drive));
    }

    public Pose2d getBranch(int direction, boolean isLeft){
        int redId = (direction+3)%6+6;
        int id = Robot.getAlliance() == Alliance.Blue ? redId + 11: redId;
        return REEF.fromAprilTagIdAndPose(id, isLeft).pose;
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

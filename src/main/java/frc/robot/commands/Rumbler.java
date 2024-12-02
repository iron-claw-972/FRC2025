// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gpm.Shooter;

public class Rumbler extends Command {
  /** Creates a new Rumbler. */
  Consumer<Boolean> consumer;
  Shooter shooter;
  public Rumbler(Shooter shooter,Consumer<Boolean> consumer) {
    this.consumer = consumer;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.atSetpoint()){
      consumer.accept(true);
    }
    else{
      consumer.accept(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

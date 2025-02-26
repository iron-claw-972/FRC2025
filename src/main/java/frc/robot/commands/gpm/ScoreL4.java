// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL4 extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public ScoreL4(Elevator elevator, Outtake outake) {
    // Add your commands in the addCommands() call, e.g.
    // TODO add move elevaotr stow setpoint later
    addCommands(
      new ParallelCommandGroup(new OuttakeCoralBasic(outake, ()-> elevator.getSetpoint() > ElevatorConstants.L3_SETPOINT + 0.001),
      new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT+0.07)),
      new MoveElevator(elevator, ElevatorConstants.STOW_SETPOINT)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.gpm.IntakeNote;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.StorageIndex;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeWithRumble extends SequentialCommandGroup {
  /** Creates a new IntakeWithRumble. */
  public IntakeWithRumble(Intake intake, StorageIndex storageIndex, Arm arm, Consumer<Boolean> rumble) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(intake, storageIndex, arm);
    addCommands(
      new IntakeNote(intake, storageIndex, arm, rumble),
      new InstantCommand(()->rumble.accept(true)),
      new WaitCommand(0.3),
      new InstantCommand(()->rumble.accept(false))
    );
  }
}

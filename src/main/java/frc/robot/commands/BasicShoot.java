// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.gpm.IntakeNote;
import frc.robot.commands.gpm.PrepareShooter;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;

/** Add your docs here. */
public class BasicShoot extends ParallelRaceGroup{
    public BasicShoot(Shooter shooter, Intake intake, StorageIndex index, Arm arm, double target, Consumer<Boolean> consumer){   
    super(
        new PrepareShooter(shooter, 1750),
        new IntakeNote(intake, index,arm, consumer)
    );
}
}

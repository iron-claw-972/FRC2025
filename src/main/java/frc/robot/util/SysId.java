// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import java.util.function.Consumer;


/** Add your docs here. */
public class SysId {

    SysIdRoutine sysIdRoutine;

    public SysId(String name, Consumer<Measure<Voltage>> driveConsumer, Consumer<SysIdRoutineLog> logConsumer, Subsystem subsystem, Config config){
        sysIdRoutine = new SysIdRoutine(
            config,
            new Mechanism(
                driveConsumer,
                logConsumer,
                subsystem,
                name
            )
        );
    }
    public SysId(String name, Consumer<Measure<Voltage>> driveConsumer, Subsystem subsystem, Config config){
        this(name,driveConsumer,null,subsystem,config);
    }

    public Command runQuasisStatic(Direction direction){
        return sysIdRoutine.quasistatic(direction);
    }
    public Command runDynamic(Direction direction){
        return sysIdRoutine.dynamic(direction);
    }
}

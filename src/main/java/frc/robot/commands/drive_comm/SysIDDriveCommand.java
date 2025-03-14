// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_comm;


import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.SysId;

/**
 * A command to run all 4 SysId routines on the drivetrain
*/
public class SysIDDriveCommand extends SequentialCommandGroup {

    private Config config = new Config();
    private SysId sysId;
    public SysIDDriveCommand(Drivetrain drive) {
        config = new Config(
            Units.Volts.of(0.5).per(Units.Seconds),
            Units.Volts.of(3),
            Units.Seconds.of(5),
            (x)->SignalLogger.writeString("state", x.toString())
        );
        Rotation2d[] angles = {
            Rotation2d.fromDegrees(0),//-45-180
            Rotation2d.fromDegrees(0),//45
            Rotation2d.fromDegrees(0),//45+180
            Rotation2d.fromDegrees(0),//-45
        };
        sysId = new SysId(
            "Drivetrain",
            x ->{
                    drive.setAngleMotors(angles);
                    drive.setDriveVoltages(x);
                },
            drive,
            config
        );
        addCommands(
            sysId.runQuasisStatic(Direction.kForward),
            new WaitCommand(0.5),
            sysId.runQuasisStatic(Direction.kReverse),
            new WaitCommand(0.5),
            sysId.runDynamic(Direction.kForward),
            new WaitCommand(0.5),
            sysId.runDynamic(Direction.kReverse)
            );
    }

    

}

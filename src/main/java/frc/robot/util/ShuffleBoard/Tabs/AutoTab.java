// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.auto_comm.FollowPathCommand;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

/**
 * Class for storing and updating information on the auto tab in Shuffleboard
*/
public class AutoTab extends ShuffleBoardTabs {

    private final SendableChooser<Command> autoCommand = new SendableChooser<>();

    private Drivetrain drive;
    private Elevator elevator;
    private Outtake outtake;

    public AutoTab(Drivetrain drive, Elevator elevator, Outtake outtake){
        this.drive = drive;
        this.elevator = elevator;
        this.outtake = outtake;
    }
    
    public void createEntries(){         
        tab = Shuffleboard.getTab("Auto");
        autoCommand.setDefaultOption("Do nothing", new DoNothing());

        // autoCommand.addOption("Center to G", new FollowPathCommand("Center to G", true, drive)
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator)));

        autoCommand.addOption("test", new FollowPathCommand("test", true, drive));
        autoCommand.addOption("Trial 2", new FollowPathCommand("Trial 2", true, drive));
        
        autoCommand.addOption("Trial", new FollowPathCommand("Trial", true, drive));
        autoCommand.addOption("Tester", new FollowPathCommand("Tester", true, drive));
        autoCommand.addOption("Position Checker", new FollowPathCommand("Position Checker", true, drive));
        autoCommand.addOption("test_2", new FollowPathCommand("test_2", true, drive));
        
        // autoCommand.addOption("#1", new FollowPathCommand("#1", true, drive)
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator))
        // .andThen(new FollowPathCommand("#2", true, drive)));

        // autoCommand.addOption("#1", new FollowPathCommand("#1", true, drive)
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator))
        // .andThen(new FollowPathCommand("#2", true, drive))
        // .andThen(new FollowPathCommand("#3", true, drive))
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator)));

        autoCommand.addOption("#1", new FollowPathCommand("#1", true, drive)
        .andThen(new FollowPathCommand("#2", true, drive))
        .andThen(new FollowPathCommand("#3", true, drive))
        .andThen(new FollowPathCommand("#4", true, drive))
        .andThen(new FollowPathCommand("#5", true, drive)));

        autoCommand.addOption("Copy of #1", new FollowPathCommand("Copy of #1", true, drive)
        .andThen(new FollowPathCommand("Copy of #2", true, drive))
        .andThen(new FollowPathCommand("Copy of #3", true, drive))
        .andThen(new FollowPathCommand("Copy of #4", true, drive))
        .andThen(new FollowPathCommand("Copy of #5", true, drive)));
        
        // autoCommand.addOption("#1", new FollowPathCommand("#1", true, drive)
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake,elevator))
        // .andThen(new FollowPathCommand("#2", true, drive))
        // .andThen(new FollowPathCommand("#3", true, drive))
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator))
        // .andThen(new FollowPathCommand("#4", true, drive))
        // .andThen(new FollowPathCommand("#5", true, drive))
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator)));


        autoCommand.addOption("Copy #1", new FollowPathCommand("Copy #1", true, drive)
        .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        .andThen(new OuttakeCoral(outtake,elevator))
        .andThen(new FollowPathCommand("Copy #2", true, drive))
        .andThen(new FollowPathCommand("Copy #3", true, drive))
        .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        .andThen(new OuttakeCoral(outtake, elevator))
        .andThen(new FollowPathCommand("Copy #4", true, drive))
        .andThen(new FollowPathCommand("Copy #5", true, drive))
        .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        .andThen(new OuttakeCoral(outtake, elevator)));

        


        autoCommand.addOption("Path with Wait and Parallel Commands", new FollowPathCommand("Path1", true, drive)
        .andThen(new WaitCommand(2.0)) // Wait for 2 seconds after the first path
        .andThen(new ParallelCommandGroup(
                new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT), // Move elevator while waiting
                new OuttakeCoral(outtake, elevator) // Parallel action for outtake
            ))
        .andThen(new FollowPathCommand("Path2", true, drive)));


        // Sequential path with multiple commands, including wait
        autoCommand.addOption("Sequential Path with Parallel Actions", new SequentialCommandGroup(
            new FollowPathCommand("Path1", true, drive),
            new WaitCommand(1.5), // Wait for 1.5 seconds after following the first path
            new ParallelCommandGroup(
                new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT), // Move the elevator in parallel
                new OuttakeCoral(outtake, elevator)),
            new FollowPathCommand("Path2", true, drive)));


        autoCommand.addOption("Path with Wait", new FollowPathCommand("Path1", true, drive)
        .andThen(new WaitCommand(2.0)) // Wait for 2 seconds after path
        .andThen(new FollowPathCommand("Path2", true, drive)));

        

        System.out.println("Hello");

        

                        
    //     autoCommand.addOption("Sequential_1",
    //     Commands.sequence(
    //         new FollowPathCommand("#1", true, drive),
    //         new InstantCommand(() -> System.out.println("Instant command."))
    //     )
    // );

        tab.add(autoCommand);
    }

    public void update(){
    }

    public SendableChooser<Command> getChooser(){
        return autoCommand;
    }
}



/**
 * // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import javax.sound.midi.VoiceStatus;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.auto_comm.FollowPathCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

 Add your docs here. 
public class AutoTab extends ShuffleBoardTabs {

    private final SendableChooser<Command> autoCommand = new SendableChooser<>();

    private Drivetrain drive;

    public AutoTab(Drivetrain drive){
        this.drive = drive;
    }
    public void intake(CANSparkFlex motor1, double speed){
        motor1.set(speed);
    }
    public void LineIntake(){
        new ParallelCommandGroup(
            new FollowPathCommand("Line", true, drive),
            new InstantCommand(() -> intake(new CANSparkFlex(5, MotorType.kBrushless), -0.5)));
    }
    public void createEntries(){  
        tab = Shuffleboard.getTab("Auto");
        
        autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
        autoCommand.addOption("Example Path", new FollowPathCommand("Example Path",true, drive));
        autoCommand.addOption("7 Piece Auto Realistic", new FollowPathCommand("7 Piece Auto Realistic",true, drive));
        autoCommand.addOption("7 Piece Auto", new FollowPathCommand("7 Piece Auto",true, drive));       
        autoCommand.addOption("Bottom 4 Piece 4 5 (No Shooting On The Move)", new FollowPathCommand("Bottom 4 Piece 4 5 (No Shooting On The Move)",true, drive));       
        autoCommand.addOption("Dream Bottom 4 Piece 4 5", new FollowPathCommand("Dream Bottom 4 Piece 4 5",true, drive));
        autoCommand.addOption("Test", new FollowPathCommand("Test", true, drive));
        autoCommand.addOption("Test 2", new FollowPathCommand("Test 2", true, drive));
        autoCommand.addOption("Test with Rotations", new FollowPathCommand("Test", true, drive));
        autoCommand.addOption("Line", new InstantCommand(() -> LineIntake()));

        // Example of running multiple commands.
        autoCommand.addOption("Multi",
            Commands.sequence(
                new FollowPathCommand("Test", true, drive),
                new InstantCommand(() -> System.out.println("Instant command."))
            )
        );

        tab.add(autoCommand);
    }

    public void update(){
    }

    public SendableChooser<Command> getChooser(){
        return autoCommand;
    }
}

*/
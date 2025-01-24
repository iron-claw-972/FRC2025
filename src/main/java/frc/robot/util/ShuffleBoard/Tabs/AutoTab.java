// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
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

        autoCommand.addOption("Center to G", new FollowPathCommand("Center to G", true, drive)
        .andThen(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT))
        .andThen(new OuttakeCoral(outtake, elevator)));

        autoCommand.addOption("test", new FollowPathCommand("test", true, drive));

         

        tab.add(autoCommand);
    }

    public void update(){
    }

    public SendableChooser<Command> getChooser(){
        return autoCommand;
    }
}

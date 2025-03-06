// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;


import java.io.IOException;
import java.util.List;


import org.json.simple.parser.ParseException;


import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto_comm.FollowPathCommand;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.commands.gpm.OuttakeCoralBasic;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;


/**
 * Class for storing and updating information on the auto tab in Shuffleboard
*/
public class AutoTab extends ShuffleBoardTabs {
    // TODO: Remove warnings
    
    private final SendableChooser<Command> autoCommand = new SendableChooser<>();

    private Drivetrain drive;
    private Elevator elevator;
    private Outtake outtake;
    private Intake intake;


    public AutoTab(Drivetrain drive, Elevator elevator, Outtake outtake, Intake intake) {
        this.drive = drive;
        this.elevator = elevator;
        this.outtake = outtake;
        this.intake = intake;
    }

    
    public void createEntries(){         
        tab = Shuffleboard.getTab("Auto");
        try {
            List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Blue Right Side");
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
        try {
            List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Right Side");
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
        try {
            List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Right Side Mirrored");
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
        autoCommand.setDefaultOption("Right Side", new PathPlannerAuto("Right Side"));

        
        autoCommand.addOption("Wait", new PathPlannerAuto("Wait Test"));

        autoCommand.addOption("Right Side", new PathPlannerAuto("Right Side"));
        autoCommand.addOption("Left Side", new PathPlannerAuto("Right Side Mirrored"));
        autoCommand.addOption("Left Side Ground", new PathPlannerAuto("Blue Right Side"));

       
        // autoCommand.addOption("#1", new FollowPathCommand("#1", true, drive)
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator))
        // .andThen(new FollowPathCommand("#2", true, drive))
        // .andThen(new FollowPathCommand("#3", true, drive))
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator))
        // .andThen(new FollowPathCommand("#4", true, drive))
        // .andThen(new FollowPathCommand("#5", true, drive))
        // .andThen(new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT))
        // .andThen(new OuttakeCoral(outtake, elevator)));    

        
        if(elevator != null && outtake != null) {
         autoCommand.addOption("WaitTest", new FollowPathCommand("Tester", true, drive)
         .andThen(new OuttakeCoralBasic(outtake, ()->true))
         .andThen(new WaitCommand(3))
         .andThen(new FollowPathCommand("Next Tester", true, drive))
         );

          autoCommand.addOption("Center to G", new FollowPathCommand("Center to G", true, drive)
         .andThen(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT))
         .andThen(new OuttakeCoral(outtake, elevator)));

         autoCommand.addOption("Center to H", new FollowPathCommand("Center to H", true, drive)
         .andThen(new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT))
         .andThen(new OuttakeCoral(outtake, elevator)));
        }
        tab.add(autoCommand);
    }

    public void update(){
    }

    public SendableChooser<Command> getChooser(){
        return autoCommand;
    }
}

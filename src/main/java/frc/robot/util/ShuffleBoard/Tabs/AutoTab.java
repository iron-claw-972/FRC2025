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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.auto_comm.FollowPathCommand;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.commands.gpm.OuttakeCoralBasic;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
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
    private Intake intake;


    public AutoTab(Drivetrain drive, Elevator elevator, Outtake outtake, Intake intake) {
        this.drive = drive;
        this.elevator = elevator;
        this.outtake = outtake;
        this.intake = intake;
    }

    
    public void createEntries(){         
        tab = Shuffleboard.getTab("Auto");
        autoCommand.setDefaultOption("Do nothing", new DoNothing());

        try {
                List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Wait Test");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
            }

        autoCommand.addOption("Wait", new PathPlannerAuto("Wait Test"));

        autoCommand.addOption("RSide Command Grouped 3 Piece", new PathPlannerAuto("RSide Command Grouped 3 Piece"));
        autoCommand.addOption("RSide Ground", new PathPlannerAuto("RSide Ground"));
        autoCommand.addOption("BSide Ground", new PathPlannerAuto("BSide Ground"));
        autoCommand.addOption("Command Grouped 3 Piece", new PathPlannerAuto("Command Grouped 3 Piece"));

        if(elevator != null && outtake != null) {
         autoCommand.addOption("WaitTest", new FollowPathCommand("Tester", true, drive)
         .andThen(new OuttakeCoralBasic(outtake))
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

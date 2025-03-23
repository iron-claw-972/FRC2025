// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard;

import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.util.Vision;
import frc.robot.util.ShuffleBoard.Tabs.AutoTab;
import frc.robot.util.ShuffleBoard.Tabs.ElevatorTab;
import frc.robot.util.ShuffleBoard.Tabs.SwerveTab;
import frc.robot.util.ShuffleBoard.Tabs.VisionTab;

/** Add your docs here. */
public class ShuffleBoardManager {

    private ArrayList<ShuffleBoardTabs> tabs = new ArrayList<>();
    
    private Field feild;

    private SwerveTab swerveTab;
    private AutoTab autoTab;
    private VisionTab visionTab;
    private ElevatorTab elevatorTab;

    public ShuffleBoardManager(Drivetrain drive, Vision vision, Elevator elevator, Outtake outtake){
        
        swerveTab = new SwerveTab(drive);
        // autoTab = new AutoTab(drive, elevator, outtake);
        // visionTab = new VisionTab(drive, vision);
        // elevatorTab = new ElevatorTab(elevator);
        tabs.add(swerveTab);
        // tabs.add(autoTab);
        // tabs.add(visionTab);
        // tabs.add(elevatorTab);

        for (ShuffleBoardTabs tab : tabs){
            tab.createEntries();        
        }
        
        // feild = new Field(drive, vision);
    }

    public void update(){
        for (ShuffleBoardTabs tab : tabs){
            tab.update();
        }
        feild.updateFeild();
    }

    // public Command getSelectedCommand(){
    //     return autoTab.getChooser().getSelected();
    // }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.util.Vision;
import frc.robot.util.ShuffleBoard.Tabs.AutoTab;

/** Add your docs here. */
public class ShuffleBoardManager {

    private ArrayList<ShuffleBoardTabs> tabs = new ArrayList<>();
    
    private Field field;

    // private SwerveTab swerveTab;
    private AutoTab autoTab;
    // private VisionTab visionTab;

    public ShuffleBoardManager(Drivetrain drive, Vision vision, Elevator elevator, Outtake outtake, Intake intake){
        
        //swerveTab = new SwerveTab(drive);
        autoTab = new AutoTab(drive, elevator, outtake, intake);
        //visionTab = new VisionTab(drive, vision);
        //tabs.add(swerveTab);
        tabs.add(autoTab);
        //tabs.add(visionTab);

        for (ShuffleBoardTabs tab : tabs){
            tab.createEntries();
        }
        
        if(RobotBase.isSimulation()){
            field = new Field(drive, vision);
        }
    }

    public void update(){
        for (ShuffleBoardTabs tab : tabs){
            tab.update();
        }
        if(field != null){
            field.updateFeild();
        }
    }

    public Command getSelectedCommand(){
        return autoTab.getChooser().getSelected();
    }
}

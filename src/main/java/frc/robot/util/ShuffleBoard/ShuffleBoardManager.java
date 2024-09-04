// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard;

import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.util.Vision;
import frc.robot.util.ShuffleBoard.Tabs.AutoTab;
import frc.robot.util.ShuffleBoard.Tabs.SwerveTab;
import frc.robot.util.ShuffleBoard.Tabs.VisionTab;

/** Add your docs here. */
public class ShuffleBoardManager {

    private ArrayList<ShuffleBoardTabs> tabs = new ArrayList<>();
    
    private Field feild;

    //private SwerveTab swerveTab;
    private AutoTab autoTab;
    private VisionTab visionTab;

    public ShuffleBoardManager(Drivetrain drive, Vision vision, Shooter shooter, Arm arm, StorageIndex index, Intake intake){
        
        // swerveTab = new SwerveTab(drive);
        autoTab = new AutoTab(drive, shooter, arm, index, intake);
         //swerveTab = new SwerveTab(drive);
        visionTab = new VisionTab(drive, vision);
         //tabs.add(swerveTab);
        tabs.add(autoTab);
        tabs.add(visionTab);

        for (ShuffleBoardTabs tab : tabs){
            tab.createEntries();
        }
        
        feild = new Field(drive);
    }

    public void update(){
        for (ShuffleBoardTabs tab : tabs){
            tab.update();
        }
        feild.updateFeild();
    }

    public Command getSelectedCommand(){
        return autoTab.getChooser().getSelected();
    }
}

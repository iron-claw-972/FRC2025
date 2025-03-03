// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.vision.AimAtTag;
import frc.robot.commands.vision.CalculateStdDevs;
import frc.robot.commands.vision.ReturnData;
import frc.robot.subsystems.Drive.Drivetrain;
import frc.robot.util.Vision;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

/** Add your docs here. */
public class VisionTab extends ShuffleBoardTabs {

    private Drivetrain drive;
    private Vision vision;

    public VisionTab(Drivetrain drive, Vision vision){
        this.drive = drive;
        this.vision = vision;
    }

    public void createEntries(){
        tab = Shuffleboard.getTab("Vision");
        addCommands(tab);         
    }

    public void update(){

    }

    public void addCommands(ShuffleboardTab tab){
        tab = Shuffleboard.getTab("Vision");
        if(vision != null){
            tab.add("Calculate std devs", new CalculateStdDevs(1000, vision, drive));
            tab.add("Return data", new ReturnData(vision));
        }
        tab.add("Aim at tag", new AimAtTag(drive));
    }

}

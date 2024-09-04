// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.vision.AimAtTag;
import frc.robot.commands.vision.AlignToTag;
import frc.robot.commands.vision.CalculateStdDevs;
// import frc.robot.commands.vision.CalculateStdDevs;
// import frc.robot.commands.vision.ReturnData;
// import frc.robot.commands.vision.TestVisionDistance;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

/** Add your docs here. */
public class VisionTab extends ShuffleBoardTabs {

    Drivetrain drive;
    Vision vision;

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
        //SmartDashboard.putData("Calculate vision std devs", new CalculateStdDevs(1000, vision, drive));
        tab.add("Calculate std devs", new CalculateStdDevs(1000, vision, drive));
      //  SmartDashboard.putData("Vision aim at tag", new AimAtTag(drive));
        tab.add("Aim at tag", new AimAtTag(drive));
        // SmartDashboard.putData("Vision distance test (forward)", new TestVisionDistance(0.1, drive, vision));
        // tab.add("Distance test (forward)", new TestVisionDistance(0.2, drive, vision));
        // SmartDashboard.putData("Vision distance test (backward)", new TestVisionDistance(-0.1, drive, vision));
        // tab.add("Distance test (backward)", new TestVisionDistance(-0.2, drive, vision));
        SmartDashboard.putData("Vision align to tag", new AlignToTag(drive));
        tab.add("Align to tag", new AlignToTag(drive));
        // SmartDashboard.putData("Return vision data", new ReturnData(vision));
        // tab.add("Return data", new ReturnData(vision));
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.gpm.Elevator;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

/** Add your docs here. */
public class ElevatorTab extends ShuffleBoardTabs {

    private Elevator elevator;
    private GenericEntry setpoint;

    public ElevatorTab(Elevator elevator){
        this.elevator = elevator;
    }

    public void createEntries(){
        tab = Shuffleboard.getTab("Elevator");
        addCommands(tab);
    }

    public void update(){
        if (RobotBase.isSimulation()){
        elevator.setSetpoint(setpoint.getDouble(0));
        }
    }

    public void addCommands(ShuffleboardTab tab){
        if(RobotBase.isSimulation()){
            tab.add("Elevator", elevator.getMechanism2d());
            setpoint = tab.add("Elevater setpoint", 0).getEntry();
        }
    }

}

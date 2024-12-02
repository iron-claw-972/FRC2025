// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public abstract class  ShuffleBoardTabs {
    protected ShuffleboardTab tab;

    public abstract void createEntries();

    public abstract void update();

    public double truncate(double value){
        return Math.floor(value*1000)/1000;
    }
}

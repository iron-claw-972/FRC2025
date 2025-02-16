// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.lang.Thread.State;

/** Add your docs here. */

public class GamePeiceState{

    public static enum STATE{
        NONE,
        INTAKE,
        INDEXER,
        OUTAKE;
    }

    private static STATE gamePeiceState = STATE.NONE;

    private GamePeiceState(){}

    public static STATE getGamePeiceState(){
        return gamePeiceState;
    }

    public static void setGamePeiceState(STATE gamePeiceState){
        GamePeiceState.gamePeiceState = gamePeiceState;
    }
}

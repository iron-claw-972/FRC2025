package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.ctre.phoenix.led.CANdle;

public class LED extends SubsystemBase {




    private CANdle candle;


    //constructor
    public LED() {
        this.candle = new CANdle(Constants.CANDLE_ID, Constants.CANDLE_BUS);

    }

    @Override
    public void periodic() {
        candle.setLEDs(0, 0, 255);

    }

    // public enum Colors {
    //     RED({255, 0, 0,}),
    //     GREEN({0, 255, 0}),
    //     BLUE({0, 0, 255}),
    //     ORANGE({252, 111, 3}),
    //     OFF({0, 0, 0});

    //     private int[] colorValue;

    //     private Colors(int[] colorValue) {
    //         this.colorValue = colorValue;
    //     }
    // }
}

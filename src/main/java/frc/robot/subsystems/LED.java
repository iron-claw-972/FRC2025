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

    public void setSection(int r, int g, int b, int start, int end){
        candle.setLEDs(r, g, b, 0, start, end);
    }
    public void alternate(int r1, int g1, int b1, int r2, int g2, int b2, int size, int offset, int total){
        for(int i = -offset; i < total; i+=size){
            boolean color2 = ((i-offset)/size)%2==0;
            if(color2){
                setSection(r2, g2, b2, i, i+size-1);
            }else{
                setSection(r1, g1, b1, i, i+size-1);
            }
        }
    }
}

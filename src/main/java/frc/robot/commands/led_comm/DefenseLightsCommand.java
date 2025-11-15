package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;

public class DefenseLightsCommand extends Command{
    private int counter;
    private int startOffset;
    private int length;
    private LED led;

    public DefenseLightsCommand(LED led, int startOffset, int length){
        this.led = led;
        this.startOffset = startOffset;
        this.length = length;

        addRequirements(led);
    }

    public void initialize(){
        counter = 0;
    }

    public void execute(){
        counter++;

        if(counter == 1){
            //setLEDs(255, 0, 0);
            led.alternate(255, 0, 0, 0, 0, 255, 5, startOffset, length);
        }else if(counter == 20){
            //setLEDs(0, 0, 255);
            led.alternate(0, 0, 255, 255, 0, 0, 5, startOffset, length);
        }
        if(counter >= 40){
            counter = 0;
        }
    
    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted){

    }
}

package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;

public class ChangeModeCommand extends Command{
    private int mode;
    private LED led;
    public ChangeModeCommand(int mode, LED led){
        this.mode = mode;
        this.led = led;

        addRequirements(led);
    }
    public void initialize(){
        mode++;
    }
    public void execute(){
        if(mode == 0){
            led.setLEDs(0, 0, 0);
        }else if(mode == 1){
            led.setLEDs(0, 130, 12);
        }else if(mode == 2){
            led.setLEDs(255, 255, 255);
        }else{
            mode = 0;
        }
    }
    public boolean isFinished(){
        return false;
    }
    public void end(){

    }
}

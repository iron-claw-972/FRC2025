package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;

public class Paint extends Command{
    private int start;
    private int end;
    private LED led;

    public Paint(LED led, int start, int end){
        this.led = led;
        this.end = end;
        this.start = start;
    }
    public void initialize(){
        led.setSection(0, 0, 255, start, end);
        System.out.println("Initialized");
    }
    public void execute(){
        System.out.println("Executing");
    }
    public boolean isFinished(){
        return false;
    }
    public void end(boolean interrupted){

    }
}

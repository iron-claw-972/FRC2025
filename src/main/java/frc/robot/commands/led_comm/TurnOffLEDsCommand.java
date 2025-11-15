package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;

public class TurnOffLEDsCommand extends Command{
    private LED led;
    public TurnOffLEDsCommand(LED led){
        this.led = led;

        addRequirements(led);
    }
    public void initialize(){
        led.setLEDs(0, 0, 0);
    }
    public void execute(){

    }
    public boolean isFinished(){
        return false;
    }
    public void end(boolean interrupted){

    }
}

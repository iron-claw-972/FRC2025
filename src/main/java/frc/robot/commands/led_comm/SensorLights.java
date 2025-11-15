package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LaserCAN.Sensor;

public class SensorLights extends Command{
    private Sensor sensor;
    private LED led;
    public SensorLights(LED led, Sensor sensor){
        this.led = led;
        this.sensor = sensor;
    }
    public void initialize(){

    }
    public void execute(){
        if(sensor.detected() == true){
            led.setSection(0, 255, 0, 8, 66);
        }else{
            led.setSection(255, 0, 0, 8, 66);
        }
    }
    public boolean isFinished(){
        return false;
    }
    public void end(boolean interrupted){

    }
}

package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LaserCAN.Sensor;

public class LEDSensorCommand extends Command{
    private LED led;
    private Sensor sensor;
    private int alliance = 0;

    public LEDSensorCommand(LED led, Sensor sensor, int alliance){
        this.led = led;
        this.sensor = sensor;

        addRequirements(led, sensor);
    }

    @Override
    public void execute() {
        if (sensor.detected()){
            led.setLEDs(0, 255, 0);
        }
        else if(alliance == 0){
            led.setLEDs(0, 0, 255);
        }else{
            led.setLEDs(255, 0, 0);
        }
    }

}

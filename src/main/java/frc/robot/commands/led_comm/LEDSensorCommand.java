package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LaserCAN.Sensor;

public class LEDSensorCommand extends Command{
    private LED led;
    private Sensor sensor;

    public LEDSensorCommand(LED led, Sensor sensor){
        this.led = led;
        this.sensor = sensor;
    }

    @Override
    public void execute() {
        if (sensor.detected()){
            led.setLEDs(0, 255, 0);
        }
        else{
            led.setLEDs(255, 0, 0);
        }
    }

}

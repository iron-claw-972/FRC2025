package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LaserCAN.Sensor;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class LEDCommand extends Command {
    private LED led;
    private Sensor sensor;
    private Drivetrain drivetrain;
    private double climbYCoordinate = 10.0;

    public LEDCommand(LED led, Sensor sensor, Drivetrain drivetrain){
        this.led = led;
        this.sensor = sensor;
        this.drivetrain = drivetrain;

        addRequirements(led);
    }

    @Override
    public void execute(){
        if (climbAligned()){
            led.setLEDs(0, 50, 0);
        }
        else if (sensor.detected()){
            led.setLEDs(0, 0, 50);
        }
    }

    private boolean climbAligned(){
        double yCoordinate = drivetrain.getPose().getY();
        return Math.abs(yCoordinate - climbYCoordinate) < 3.0;
    }
}

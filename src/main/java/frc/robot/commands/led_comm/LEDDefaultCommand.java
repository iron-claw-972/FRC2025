package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LaserCAN.Sensor;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.Vision;

public class LEDDefaultCommand extends Command {
    private Vision vision;
    private LED led;
    private Sensor sensor;
    private Drivetrain drivetrain;
    private double climbYCoordinate = 10.0;
    private boolean allianceIsRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    public LEDDefaultCommand(LED led, Sensor sensor, Drivetrain drivetrain, Vision vision){
        this.led = led;
        this.sensor = sensor;
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(led);
    }

    @Override
    public void execute(){
        if (climbAligned()){
            //When aligned to climb
            led.setTwoColorWave(255, 0, 100, 100, 0, 255);
        }else if (sensor.detected()){
            //When sensor detected
            led.setLEDs(0, 255, 0);
        }else if(vision.getCamerasConnected()){

        }else if(allianceIsRed){
            led.setTwoColorWave(255, 255, 255, 255, 0, 0);
        }else{
            led.setTwoColorWave(255, 255, 255, 0, 0, 255);
        }
    }

    private boolean climbAligned(){
        double yCoordinate = drivetrain.getPose().getY();
        return Math.abs(yCoordinate - climbYCoordinate) < 3.0;
    }
}

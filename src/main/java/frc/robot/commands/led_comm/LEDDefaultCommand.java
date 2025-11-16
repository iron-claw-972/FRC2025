package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.Vision.Vision;

public class LEDDefaultCommand extends Command {
    private Vision vision;
    private LED led;
    private Outtake outtake;
    private Drivetrain drivetrain;
    //TODO: change this to actual climb coordinate
    private double climbYCoordinate = 10.0;
    private boolean allianceIsRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    public LEDDefaultCommand(LED led, Outtake outtake, Drivetrain drivetrain, Vision vision){
        this.led = led;
        this.outtake = outtake;
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(led);
    }

    @Override
    public void execute(){
        if (climbAligned()){
            //When aligned to climb
            led.setTwoColorWave(255, 0, 100, 100, 0, 255);
        }else if(vision.oneCameraDisconnected()){
            //flash if camera disconnected
            led.setStrobeLights(255, 0, 0);
        }else if (outtake.coralLoaded()){
            //When coral detected
            led.setLEDs(0, 255, 0);
        }else if(allianceIsRed){
            //Red alliance
            led.setTwoColorWave(255, 0, 0, 255, 255, 255);
        }else{
            //Blue alliance
            led.setTwoColorWave(0, 0, 255, 255, 255, 255);
        }
    }

    private boolean climbAligned(){
        double yCoordinate = drivetrain.getPose().getY();
        return Math.abs(yCoordinate - climbYCoordinate) < 3.0;
    }
}

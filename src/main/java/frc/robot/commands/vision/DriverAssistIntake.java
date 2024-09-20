package frc.robot.commands.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DetectedObject;
import frc.robot.util.Vision;

public class DriverAssistIntake extends Command {
    private Drivetrain drive;
    private BaseDriverConfig driver;
    private Vision vision;

    public DriverAssistIntake(Drivetrain drive, BaseDriverConfig driver, Vision vision){
        this.drive = drive;
        this.driver = driver;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        DetectedObject closest = vision.getBestGamePiece(Units.degreesToRadians(20));
        double xTranslation = driver.getForwardTranslation();
        double yTranslation = driver.getSideTranslation();
        if(closest == null){
            double rotation = driver.getRotation();
            drive.drive(xTranslation, yTranslation, rotation, true, false);
            return;
        }
        double angle = closest.getAngle();
        double speed = Math.hypot(xTranslation, yTranslation);
        double velocityAngle = Math.atan2(yTranslation, xTranslation);
        double parallelSpeed = speed * Math.cos(angle-velocityAngle);
        drive.driveHeading(parallelSpeed * Math.cos(angle), parallelSpeed * Math.sin(angle), angle, true);
    }

    @Override
    public void end(boolean interrupted){
        drive.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

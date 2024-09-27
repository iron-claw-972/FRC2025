package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.swerve.DriveConstants;
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
    public void initialize(){}

    @Override
    public void execute(){
        // Get driver inputs
        double xTranslation = driver.getForwardTranslation();
        double yTranslation = driver.getSideTranslation();
        // Get the closest game piece within 15 degrees of the robot's heading
        DetectedObject object = vision.getBestGamePiece(Units.degreesToRadians(60));
        // If no object is detected, drive normally
        if(object == null){
            normalDrive(xTranslation, yTranslation);
            return;
        }
        // Get field-relative angle from the robot to the object
        double angle = object.getAngle();
        // The speed the driver wants to drive at
        double speed = Math.hypot(xTranslation, yTranslation);
        // The angle the driver wants to drive at
        double velocityAngle = Math.atan2(yTranslation, xTranslation);
        // If this angle is too different from the angle to the object, drive normally
        if(Math.abs(MathUtil.angleModulus(angle-velocityAngle)) > Units.degreesToRadians(90)){
            normalDrive(xTranslation, yTranslation);
            return;
        }
        // The component of speed in the driection of the object
        double slowFactor = driver.getIsSlowMode() ? DriveConstants.kSlowDriveFactor : 1;
        double parallelSpeed = speed * Math.cos(angle-velocityAngle) * slowFactor;
        // Drive using only the parallel component of the speed
        drive.driveHeading(parallelSpeed * Math.cos(angle), parallelSpeed * Math.sin(angle), Math.PI+angle, true);
    }

    private void normalDrive(double x, double y){
        double rotation = -driver.getRotation();
        double slowFactor = driver.getIsSlowMode() ? DriveConstants.kSlowDriveFactor : 1;
        drive.drive(x * slowFactor, y * slowFactor, rotation * slowFactor, true, false);
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

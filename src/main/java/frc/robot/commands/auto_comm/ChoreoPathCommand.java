package frc.robot.commands.auto_comm;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.Drivetrain;

public class ChoreoPathCommand extends SequentialCommandGroup {
    private static AutoFactory factory;

    public ChoreoPathCommand(String pathName, boolean resetOdemetry, Drivetrain drive) {
        if(factory == null){
            factory = new AutoFactory(
                drive::getPose,
                drive::resetOdometry,
                sample -> drive.setChassisSpeeds(sample.getChassisSpeeds(), false),
                true,
                drive,
                (trajectory, bool)->{
                    if(bool){
                        //LogManager.log("Trajectory started", trajectory.name());
                    }else{
                        //LogManager.log("Trajectory ended", trajectory.name());
                    }
                }
            );
        }

        var command = factory.trajectoryCmd(pathName);

        addCommands(
            resetOdemetry ? new InstantCommand(()->factory.resetOdometry(pathName)) : new DoNothing(),
            command
        );
    }
}

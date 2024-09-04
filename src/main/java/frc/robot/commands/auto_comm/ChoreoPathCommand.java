package frc.robot.commands.auto_comm;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import java.util.Optional;

public class ChoreoPathCommand extends SequentialCommandGroup {
    private final Drivetrain drive;
    private final ChoreoTrajectory trajectory;

    public ChoreoPathCommand(String pathName, boolean resetOdemetry, Drivetrain drive) {
        this.drive = drive;
        this.trajectory = Choreo.getTrajectory(pathName);

        var command = Choreo.choreoSwerveCommand(
                trajectory,
                drive::getPose,
                drive.getXController(),
                drive.getYController(),
                drive.getRotationController(),
                (ChassisSpeeds speeds) ->
                        drive.setChassisSpeeds(speeds, false),
                ChoreoPathCommand::getShouldFlip,
                drive
                                                );

        addCommands(
                new InstantCommand(() -> resetOdemetry(resetOdemetry)),
                command
                   );
    }

    public void resetOdemetry(boolean resetOdemetry) {
        if (resetOdemetry) {
            boolean shouldFlip = getShouldFlip();
            if (shouldFlip) {
                drive.resetOdometry(trajectory.getFlippedInitialPose());
            } else {
                drive.resetOdometry(trajectory.getInitialPose());
            }
        }
    }

    public static boolean getShouldFlip() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}

package frc.robot.commands.auto_comm;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.ArmToPos;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.StorageIndex;

public class MoveArmForShoot extends SequentialCommandGroup {

    public MoveArmForShoot(String pathName, Arm arm) {
        var trajectory = Choreo.getTrajectory(pathName);

        var endPose = ChoreoPathCommand.getShouldFlip()
                              ? trajectory.getFlippedFinalPose()
                              : trajectory.getFinalPose();

        // yoink from shoot.java
        Pose3d speakerPose = Robot.getAlliance() == Alliance.Red ?
                VisionConstants.RED_SPEAKER_POSE : VisionConstants.BLUE_SPEAKER_POSE;
        // shooterHeight and shooterOffset have an additional offset because the shooter is offset from the arm, right?
        Rotation2d rotation = endPose.getRotation();
        double angleToShooter = arm.getAngleRad()+Units.degreesToRadians(28.78);
        double shooterToPivot = Units.inchesToMeters(13.651);
        double horizontalDist = ArmConstants.PIVOT_X + shooterToPivot * Math.cos(angleToShooter);
        // Set displacement to speaker
        Pose3d displacement = new Pose3d(
                endPose.getX() + horizontalDist * rotation.getCos()-speakerPose.getX(),
                endPose.getY() + horizontalDist * rotation.getSin()-speakerPose.getY(),
                // shooterHeight-speakerPose.getZ(),
                ArmConstants.PIVOT_HEIGHT + shooterToPivot * Math.sin(angleToShooter) - speakerPose.getZ(),
                new Rotation3d(
                0,
                ShooterConstants.ANGLE_OFFSET - arm.getAngleRad(),
                Math.PI + rotation.getRadians()));
                // .relativeTo(speakerPose);
                //.times(-1);

        // get the drivetrain velocities
        double v_note = ShooterConstants.SHOOT_SPEED_MPS;

        // X distance to speaker
        double x = Math.sqrt((displacement.getX() * displacement.getX())
                        // Y distance to speaker
                        + displacement.getY() * displacement.getY());
        // height (sorry that it's called y)
        double y = displacement.getZ();
        // Basic vertical angle calculation (static robot)
        double phi_v = Math.atan(Math.pow(v_note, 2) / 9.8 / x * (1 - Math.sqrt(1
                        + 19.6 / Math.pow(v_note, 2) * (y - 4.9 * x * x / Math.pow(v_note, 2)))));
        phi_v -= Units.degreesToRadians(.5);

        addCommands(
                new ArmToPos(arm, ShooterConstants.ANGLE_OFFSET - phi_v)
               );
    }
}

package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmToPos;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex; 

public class ShootKnownPos extends SequentialCommandGroup {
	
	/**
	* A preset shot that can be taken by the {@link frc.robot.commands.gpm.ShootKnownPos} command
	*/
	public enum ShotPosition {
		STAGE_ISH(ShooterConstants.ANGLE_OFFSET - .6, Shooter.addSlip(Shooter.shooterSpeedToRPM(ShooterConstants.SHOOT_SPEED_MPS))),
		// TODO: add actual values
		SUBWOOFER(ArmConstants.subwooferSetpoint, Shooter.addSlip(Shooter.shooterSpeedToRPM(ShooterConstants.SHOOT_SPEED_MPS-1.0)));
		
		private double armAngle;
		private double shooterSpeed;

		ShotPosition(double armAngle, double shooterSpeed) {
			this.armAngle = armAngle;
			this.shooterSpeed = shooterSpeed;
		}

        public double getArmAngle() {
            return armAngle;
        }

        public double getShooterSpeed() {
            return shooterSpeed;
        }
	}
	
	/**
	* This command performs a shot from a known position
	* 
	* @param shooter      the shooter to use
	* @param arm          the arm to use
	* @param storageIndex the indexer to use
	* @param shot         the preset shot to take
	*/
	public ShootKnownPos(Shooter shooter, Arm arm, StorageIndex storageIndex, ShotPosition shot) {
		addRequirements(shooter, arm, storageIndex);

		addCommands(
			new ParallelCommandGroup(
				new ArmToPos(arm, shot.getArmAngle()),  
				new PrepareShooter(shooter, 2000)),  
				new WaitCommand(0.5),
			new IndexerFeed(storageIndex),
			new ParallelCommandGroup(
				new InstantCommand(() -> arm.setAngle(ArmConstants.stowedSetpoint)),
				new PrepareShooter(shooter, 0)));

	}
}


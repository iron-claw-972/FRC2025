// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.vision.AimAtTag;
import frc.robot.commands.vision.CalculateStdDevs;
import frc.robot.commands.vision.ReturnData;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

/** Add your docs here. */
public class VisionTab extends ShuffleBoardTabs {

    private Drivetrain drive;
    private Vision vision;

    public VisionTab(Drivetrain drive, Vision vision){
        this.drive = drive;
        this.vision = vision;
    }

    public void createEntries(){
        tab = Shuffleboard.getTab("Vision");
        addCommands(tab);         
    }

    public void update(){

    }

    public void addCommands(ShuffleboardTab tab){
        tab = Shuffleboard.getTab("Vision");
        if(vision != null){
            tab.add("Calculate std devs", new CalculateStdDevs(1000, vision, drive));
            tab.add("Return data", new ReturnData(vision));
        }

		tab.add("Shutdown OrangePi", new InstantCommand(() -> {
			if (Robot.isSimulation()) {
				// this will probably break on Windows systems so...
				System.out.println("What OrangePi? This is simulation!");
			} else {
				try {
					String[] command = new String[]{"sshpass", "-praspberry", "ssh",
						"-o", "UserKnownHostsFile /dev/null",
						"-o", "StrictHostKeyChecking no",
						"pi@10.9.72.12", "sudo", "shutdown", "now"};
					Runtime.getRuntime().exec(command);
				} catch (Exception e) {
					String message = e.getMessage() == null ? "unknown" : e.getMessage();
					System.out.println("Failed to shutdown OrangePi. Reason: " + message);
				}
			}
			}).ignoringDisable(true)
		);
        tab.add("Aim at tag", new AimAtTag(drive));
    }

}

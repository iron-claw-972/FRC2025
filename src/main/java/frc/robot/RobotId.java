package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Set of known Robot Names.
 * <p>The name of a robot in the RoboRIO's persistent memory.
 * At deploy time, that name is used to set the corresponding RobotId.
 * <p>Note that the RobotId is determined at Deploy time.
 */
public enum RobotId {
    Default,
    SwerveCompetition, Vertigo, Vivace, Phil, BetaBot,
    ClassBot1, ClassBot2, ClassBot3, ClassBot4,
    TestBed1, TestBed2;

    /** The key used to access the RobotId name in the RoboRIO's persistent memory. */
    public static final String ROBOT_ID_KEY = "RobotId";

    /**
     * Is this robot a classbot?
     * @return true if a classbot
     * @deprecated this method is not needed....
     */
    @Deprecated
    public boolean isClassBot() {
        return this == ClassBot1 || this == ClassBot2 || this == ClassBot3 || this == ClassBot4;
    }

    /**
     * Whether this robot is a swerve bot
     * @return true if a swerve bot
     * @deprecated this method is not needed....
     */
    @Deprecated
    public boolean isSwerveBot() {
        return this == SwerveCompetition || this == Phil || this == Vertigo || this == Vivace || this == BetaBot;
    }
    
    /**
     * Determine the Robot Identity from the RoboRIO's onboard Preferences (flash memory).
     * @returns the RobotId
     */
    public static RobotId getRobotId() {
        // assume a default identity
        RobotId robotId = RobotId.Default;

        // check whether Preferences has an entry for the RobotId
        if (!Preferences.containsKey(ROBOT_ID_KEY)) {
            // There is no such key. Set it to the default identity.
            // This step guarantees persistent memory will have a key.
            setRobotId(RobotId.Default);
        }

        // Remove the "Default" key if present.
        // This key was the result of a programming error in 2023.
        if (Preferences.containsKey("Default")) {
            Preferences.remove("Default");
        }

        // get the RobotId string from the RoboRIO's Preferences
        String strId = Preferences.getString(ROBOT_ID_KEY, RobotId.Default.name());

        // match that string to a RobotId by looking at all possible RobotId enums
        for (RobotId rid : RobotId.values()) {
            // does the preference string match the RobotId enum?
            if (strId.equals(rid.name())) {
                // yes, this instance is the desired RobotId
                robotId = rid;
                break;
            }
        }

        // report the RobotId to the SmartDashboard.
        //SmartDashboard.putString("RobotID", robotId.name());

        // return the robot identity
        return robotId;
    }
    
    /**
     * Set the RobotId in the RoboRIO's preferences (flash memory).
     * <p>
     * Calling it after the robot has been constructed (robotInit()) does not affect the robot.
     */
    static void setRobotId(RobotId robotId) {
        // Set the robot identity in the RoboRIO Preferences
        Preferences.setString(ROBOT_ID_KEY, robotId.name());
    }
}

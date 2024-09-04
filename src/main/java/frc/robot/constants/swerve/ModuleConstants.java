package frc.robot.constants.swerve;

/**
 * Container class for module constants, defined using constants from {@link DriveConstants}
 * .
 *
 * @see DriveConstants
 */
public enum ModuleConstants {

    FRONT_LEFT(
            DriveConstants.kDriveFrontLeft,
            DriveConstants.kSteerFrontLeft,
            DriveConstants.kEncoderFrontLeft,
            DriveConstants.kSteerOffsetFrontLeft,
            ModuleType.FRONT_LEFT,
            DriveConstants.DRIVE_KS_Values[0],
            DriveConstants.DRIVE_KV_Values[0],
            DriveConstants.DRIVE_KA_Values[0],
            DriveConstants.P_Values[0],
            DriveConstants.I_Values[0],
            DriveConstants.D_Values[0]
    ),
    FRONT_RIGHT(
            DriveConstants.kDriveFrontRight,
            DriveConstants.kSteerFrontRight,
            DriveConstants.kEncoderFrontRight,
            DriveConstants.kSteerOffsetFrontRight,
            ModuleType.FRONT_RIGHT,
            DriveConstants.DRIVE_KS_Values[1],
            DriveConstants.DRIVE_KV_Values[1],
            DriveConstants.DRIVE_KA_Values[1],
            DriveConstants.P_Values[1],
            DriveConstants.I_Values[1],
            DriveConstants.D_Values[1]
    ),
    BACK_LEFT(
            DriveConstants.kDriveBackLeft,
            DriveConstants.kSteerBackLeft,
            DriveConstants.kEncoderBackLeft,
            DriveConstants.kSteerOffsetBackLeft,
            ModuleType.BACK_LEFT,
            DriveConstants.DRIVE_KS_Values[2],
            DriveConstants.DRIVE_KV_Values[2],
            DriveConstants.DRIVE_KA_Values[2],
            DriveConstants.P_Values[2],
            DriveConstants.I_Values[2],
            DriveConstants.D_Values[2]
    ),
    BACK_RIGHT(
            DriveConstants.kDriveBackRight,
            DriveConstants.kSteerBackRight,
            DriveConstants.kEncoderBackRight,
            DriveConstants.kSteerOffsetBackRight,
            ModuleType.BACK_RIGHT,
            DriveConstants.DRIVE_KS_Values[3],
            DriveConstants.DRIVE_KV_Values[3],
            DriveConstants.DRIVE_KA_Values[3],
            DriveConstants.P_Values[3],
            DriveConstants.I_Values[3],
            DriveConstants.D_Values[3]
    ),

    NONE(0, 0, 0, 0.0, ModuleType.NONE,0,0,0,0,0,0);

    private final int drivePort;
    private final int steerPort;
    private final int encoderPort;
    private final double steerOffset;
    private final double ks;
    private final double kv;
    private final double ka;
    private final double driveP;
    private final double driveI;
    private final double driveD;
    private final ModuleType type;

    ModuleConstants(
            int drivePort,
            int steerPort,
            int encoderPort,
            double steerOffset,
            ModuleType type,
            double ks,
            double kv,
            double ka,
            double driveP,
            double driveI,
            double driveD

                   ) {
        this.drivePort = drivePort;
        this.steerPort = steerPort;
        this.encoderPort = encoderPort;
        this.steerOffset = steerOffset;
        this.type = type;
        this.ks =ks;
        this.kv= kv;
        this.ka = ka;
        this.driveP =driveP;
        this.driveI = driveI;
        this.driveD = driveD;
    }

    public int getDrivePort() {
        return drivePort;
    }

    public int getSteerPort() {
        return steerPort;
    }

    public int getEncoderPort() {
        return encoderPort;
    }

    public double getSteerOffset() {
        return steerOffset;
    }

    public ModuleType getType() {
        return type;
    }
    public double getKs(){
        return ks;
    }
    public double getKv(){
        return kv;
    }
    public double getKa(){
        return ka;
    }
    public double getDriveP(){
        return driveP;
    }
    public double getDriveI(){
        return driveI;
    }
    public double getDriveD(){
        return driveD;
    }

}
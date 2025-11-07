package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

import au.grapplerobotics.CanBridge;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import au.grapplerobotics.LaserCan;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.Animation;

public class LED extends SubsystemBase {

    private CANdle candle;
    public static final int stripLength = 67;
    private LaserCan sensor;
  

    // Constructor
    public LED() {
        this.candle = new CANdle(IdConstants.CANDLE_ID, "CANivore");
        this.sensor = new LaserCan(IdConstants.LASERCAN_ID);

        candle.configStatusLedState(false);
        candle.configLOSBehavior(false);

        System.out.println("Strip type ec: " + candle.configLEDType(LEDStripType.GRB));

        candle.configBrightnessScalar(1);
        candle.configVBatOutput(VBatOutputMode.On);
        candle.configV5Enabled(true); // Turns off LEDs

        SmartDashboard.putData("set LED red", new InstantCommand(() -> setLEDs(255, 0, 0)));
        SmartDashboard.putData("set LED blue", new InstantCommand(() -> setLEDs(0, 0, 255)));
        SmartDashboard.putData("set LED green", new InstantCommand(() -> setLEDs(0, 255, 0)));
        SmartDashboard.putData("turn off LED", new InstantCommand(() -> setLEDs(0, 0, 0)));

        SmartDashboard.putData("set LED blue and green", new InstantCommand(() -> alternate(0, 255, 0, 0, 0, 255, 2, 0, 100)));

        SmartDashboard.putData("rainbow animation", new InstantCommand(() -> animate(new RainbowAnimation())));
        SmartDashboard.putData("strobe animation", new InstantCommand(() -> animate(new StrobeAnimation(255, 255, 255))));
        SmartDashboard.putData("turn off animations", new InstantCommand(() -> animate(null)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("laserCan", sensor.getMeasurement().toString());
    }

    /**
     * Sets the color of all the LEDs.
     *
     * @param red   Red value (0-255)
     * @param green Green value (0-255)
     * @param blue  Blue value (0-255)
     */
    public void setLEDs(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
    }
    public boolean sensorTriggered(){
        // if(sensor.getMeasurement() <= 20){

        // }
        return true;
    }

   /**
     * Sets an animation for the LEDs.
     *
     * @param animation The animation object (e.g., RainbowAnimation, StrobeAnimation, etc.)
     */
    public void animate(Animation animation) {
        candle.animate(animation);
    }

    /**
     * Sets the color of a specific section of LEDs.
     *
     * @param r     Red value (0-255)
     * @param g     Green value (0-255)
     * @param b     Blue value (0-255)
     * @param start Start index of the section inclusive
     * @param end   End index of the section exclusive
     */
    public void setSection(int r, int g, int b, int start, int end) {
        candle.setLEDs(r, g, b, 0, start, end-start);
    }

    /**
     * Creates an alternating pattern of two colors across the LEDs.
     *
     * @param r1     Red value of the first color (0-255)
     * @param g1     Green value of the first color (0-255)
     * @param b1     Blue value of the first color (0-255)
     * @param r2     Red value of the second color (0-255)
     * @param g2     Green value of the second color (0-255)
     * @param b2     Blue value of the second color (0-255)
     * @param size   Size of each color block
     * @param offset Offset for the starting position of the pattern
     * @param total  Total number of LEDs
     */
    public void alternate(int r1, int g1, int b1, int r2, int g2, int b2, int size, int offset, int total) {
        for (int i = -offset; i < total; i += size) {
            boolean color2 = ((i - offset) / size) % 2 == 0;
            if (color2) {
                setSection(r2, g2, b2, i, i + size);
            } else {
                setSection(r1, g1, b1, i, i + size);
            }
        }
    }
}

package frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

import com.ctre.phoenix.led.CANdle;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.Animation;

public class LED extends SubsystemBase {
    private CANdle candle;
    public static final int stripLength = 232;

    private final CANdleConfiguration config = new CANdleConfiguration();
    private int defenseCounter = 0;
    private int strobeCounter = 0;

    // Animation state
    private double waveOffset = 0;
    private final double waveSpeed = 0.08;
    private final double waveFrequency = 0.25;

    // Constructor
    public LED() {
        this.candle = new CANdle(IdConstants.CANDLE_ID, "CANivore");
        candle.configAllSettings(config);

        candle.configStatusLedState(false);
        candle.configLOSBehavior(false);

        System.out.println("Strip type ec: " + candle.configLEDType(LEDStripType.GRB));

        candle.configBrightnessScalar(1);
        candle.configVBatOutput(VBatOutputMode.On);
        candle.configV5Enabled(true); // Turns off LEDs
        setLEDs(0, 0, 0);
        setSection(0, 255, 0, 0, 4);
        setSection(255, 0, 0, 14, 16);
    }

    @Override
    public void periodic() {

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
        for (int i = offset; i < total; i += size) {
            boolean color2 = ((i - offset) / size) % 2 == 0;
            if (color2) {
                setSection(r2, g2, b2, i, i + size);
            } else {
                setSection(r1, g1, b1, i, i + size);
            }
        }
    }
    public void setTwoColorWave(int r1, int g1, int b1, int r2, int g2, int b2) {
        for (int i = 0; i < stripLength; i++) {

            double wave = (Math.sin(i * waveFrequency + waveOffset) + 1) / 2.0;
            double inverseBias = 5;     // higher = more color 2
            wave = 1 - Math.pow(1 - wave, inverseBias);

            int r = (int)(r1 * wave + r2 * (1 - wave));
            int g = (int)(g1 * wave + g2 * (1 - wave));
            int b = (int)(b1 * wave + b2 * (1 - wave));

            candle.setLEDs(r, g, b, 0, i, 1);
        }

        waveOffset += waveSpeed;
    }

    public void setStrobeLights(int r1, int g1, int b1){
        strobeCounter++;

        if(strobeCounter == 1){
            setLEDs(r1, g1, b1);
        }else if(strobeCounter == 10){
            setLEDs(0, 0, 0);
        }
        if(strobeCounter >= 20){
            strobeCounter = 0;
        }
    }
    public void defenseLights(){
        defenseCounter++;

        if(defenseCounter == 1){
            //setLEDs(255, 0, 0);
            alternate(255, 0, 0, 0, 0, 255, 5, 8, stripLength);
        }else if(defenseCounter == 20){
            //setLEDs(0, 0, 255);
            alternate(0, 0, 255, 255, 0, 0, 5, 8, stripLength);
        }
        if(defenseCounter >= 40){
            defenseCounter = 0;
        }
    }
}


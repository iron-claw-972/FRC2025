package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class LED extends SubsystemBase {

    private CANdle candle;
    private static final int stripLength = 67;

    // private CANdleConfiguration config;

    // constructor
    public LED() {
        this.candle = new CANdle(IdConstants.CANDLE_ID, "rio");
        // this.config = new CANdleConfiguration();

        // config.statusLedOffWhenActive = true;
        // config.disableWhenLOS = false;
        // config.stripType = LEDStripType.RGB;
        // config.brightnessScalar = 1;
        // config.vBatOutputMode = VBatOutputMode.Modulated;
        // config.v5Enabled = false;

        // candle.configAllSettings(config, 10000);

        candle.configStatusLedState(false);
        candle.configLOSBehavior(false);

        System.out.println("Strip type ec: " + candle.configLEDType(LEDStripType.GRB));

        candle.configBrightnessScalar(1);
        candle.configVBatOutput(VBatOutputMode.On);
        candle.configV5Enabled(true); // Turns off LEDs
    }

    @Override
    public void periodic() {
    }

    // public enum Colors {
    // RED({255, 0, 0,}),
    // GREEN({0, 255, 0}),
    // BLUE({0, 0, 255}),
    // ORANGE({252, 111, 3}),
    // OFF({0, 0, 0});

    // private int[] colorValue;

    // private Colors(int[] colorValue) {
    // this.colorValue = colorValue;
    // }
    // }

    public void setSection(int r, int g, int b, int start, int end) {
        candle.setLEDs(r, g, b, 0, start, end);
    }

    public void alternate(int r1, int g1, int b1, int r2, int g2, int b2, int size, int offset, int total) {
        for (int i = -offset; i < total; i += size) {
            boolean color2 = ((i - offset) / size) % 2 == 0;
            if (color2) {
                setSection(r2, g2, b2, i, i + size - 1);
            } else {
                setSection(r1, g1, b1, i, i + size - 1);
            }
        }
    }
}

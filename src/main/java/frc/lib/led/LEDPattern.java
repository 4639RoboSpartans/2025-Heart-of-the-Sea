package frc.lib.led;

import edu.wpi.first.wpilibj.util.Color;

public interface LEDPattern {
    LEDPattern BLANK = (led, time) -> new Color(0, 0, 0);

    Color get(int led, double time);

    static Color mixColors(Color a, Color b, double aWeight) {
        double bWeight = 1 - aWeight;
        return new Color(
            a.red * aWeight + b.red * bWeight,
            a.green * aWeight + b.green * bWeight,
            a.blue * aWeight + b.blue * bWeight
        );
    }
}

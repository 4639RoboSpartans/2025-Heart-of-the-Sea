package frc.lib.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class LEDCommandFactory {
    public Command LEDBreathingRed(){
        return new RunCommand(() -> LEDStrip.getInstance().usePattern(new LEDPattern() {
            @Override
            public Color8Bit get(int led, double time) {
                return new Color8Bit(Color.fromHSV(0, 100, (int) (75 + 25 * Math.sin(time/3))));
            }
        }));
    }

    public Command LEDBreathingBlue(){
        return new RunCommand(() -> LEDStrip.getInstance().usePattern(new LEDPattern() {
            @Override
            public Color8Bit get(int led, double time) {
                return new Color8Bit(Color.fromHSV(238, 100, (int) (75 + 25 * Math.sin(time/3))));
            }
        }));
    }
}

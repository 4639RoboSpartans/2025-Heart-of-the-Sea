package frc.lib.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SubsystemManager;

import java.util.Arrays;
import java.util.Optional;
import java.util.stream.IntStream;

public class LEDCommandFactory {
    public static Command LEDBreathingRed(){
        return new RunCommand(() -> LEDStrip.getInstance().usePattern(new LEDPattern() {
            @Override
            public Color8Bit get(int led, double time) {
                return new Color8Bit(Color.fromHSV(0, 100, (int) (75 + 25 * Math.sin(time * 2))));
            }
        }), SubsystemManager.getInstance().getLEDStripSubsystem());
    }

    public static Command LEDBreathingBlue(){
        return new RunCommand(() -> LEDStrip.getInstance().usePattern(new LEDPattern() {
            @Override
            public Color8Bit get(int led, double time) {
                return new Color8Bit(Color.fromHSV(238, 100, (int) (75 + 25 * Math.sin(time * 2))));
            }
        }), SubsystemManager.getInstance().getLEDStripSubsystem());
    }

    public static Command LEDThreeFlashGreen(){
        return new RunCommand(() -> LEDStrip.getInstance().usePattern(new LEDPattern() {
            @Override
            public Color8Bit get(int led, double time) {
                return new Color8Bit(Color.fromHSV(121, 100, Math.sin(time * 12)>0 ? 100 : 0));
            }
        }), SubsystemManager.getInstance().getLEDStripSubsystem()).withTimeout(3 * Math.PI/8);
    }

    public static Command LEDFlashRed(){
        return new RunCommand(() -> LEDStrip.getInstance().usePattern(new LEDPattern() {
            @Override
            public Color8Bit get(int led, double time) {
                return new Color8Bit(Color.fromHSV(0, 100, Math.sin(time/4)>0 ? 100 : 0));
            }
        }), SubsystemManager.getInstance().getLEDStripSubsystem());
    }

    public static Command LEDSolidColor(Color color){
        return new RunCommand(() -> LEDStrip.getInstance().usePattern(new SolidLEDPattern(new Color8Bit(color))), SubsystemManager.getInstance().getLEDStripSubsystem());
    }

    public static Command LEDThreeFlashThenSolidGreen(){
        return LEDThreeFlashGreen().andThen(LEDSolidColor(Color.kLimeGreen));
    }

    public static Command flashColors(double period, Color... colors){
        var singleColorLength = period/ colors.length;
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        Arrays.stream(colors).forEachOrdered(color -> {
            commandGroup.addCommands(LEDSolidColor(color).withTimeout(singleColorLength));
        });
        return commandGroup.repeatedly();
    }

    public static Command flashBlueOrange(){
        return flashColors(0.5, Color.kBlue, Color.kOrange);
    }

    public static Command colorProfileBetween(Color startColor, Color endColor, double transitionLength){
        double[] rgbStart = new double[]{startColor.red * 255, startColor.green * 255, startColor.blue * 255};
        double[] rgbEnd = new double[]{endColor.red * 255, endColor.green * 255, endColor.blue * 255};

        double startTime = Timer.getFPGATimestamp();

        double[] differences = IntStream.range(0, 3).mapToDouble(i -> rgbEnd[i] - rgbStart[i]).toArray();

        return new RunCommand(() -> LEDStrip.getInstance().usePattern(new LEDPattern() {

            @Override
            public Color8Bit get(int led, double time) {
                return new Color8Bit(
                        (int) (rgbStart[0] + ((time-startTime)/transitionLength) * differences[0]),
                        (int) (rgbStart[1] + ((time-startTime)/transitionLength) * differences[1]),
                        (int) (rgbStart[2] + ((time-startTime)/transitionLength) * differences[2]));
            }
        }), SubsystemManager.getInstance().getLEDStripSubsystem()).until(() -> Timer.getFPGATimestamp() >= startTime + transitionLength);
    }

    public static Command cycleBetweenColors(double period, double transitionLength, Color... colors){
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        var oneColorLength = period / colors.length;
        var solidColorLength = oneColorLength - transitionLength;
        Optional<Color> lastColor = Optional.empty();
        Arrays.stream(colors).forEachOrdered(color -> {
            lastColor.ifPresent(prevColor -> commandGroup.addCommands(colorProfileBetween(prevColor, color, transitionLength)));
            commandGroup.addCommands(LEDSolidColor(color).withTimeout(solidColorLength));
        });
        commandGroup.addCommands(colorProfileBetween(colors[colors.length-1], colors[0], transitionLength));

        return commandGroup.repeatedly();
    }

    public static Command blueOrangeCycle() {
        return cycleBetweenColors(3, 0.5, Color.kBlue, Color.kOrange);
    }

    public static Command usePattern(LEDPattern pattern){
        return new RunCommand(() -> LEDStrip.getInstance().usePattern(pattern), SubsystemManager.getInstance().getLEDStripSubsystem());
    }

    /*
        Intake coral - triple flash green -> solid green done
        Honed -> triple flash green done
        Not honed -> flash red done
        Default disabled ->breathing alliance color done
        Reef align -> flashing orange blue done
        Default -> blue orange cycling
     */
}

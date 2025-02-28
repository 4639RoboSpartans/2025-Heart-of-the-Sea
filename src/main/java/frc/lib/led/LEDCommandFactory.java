package frc.lib.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SubsystemManager;

import java.util.Arrays;

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
                return new Color8Bit(Color.fromHSV(238, 100, (int) (75 + 25 * Math.sin(time * 18))));
            }
        }), SubsystemManager.getInstance().getLEDStripSubsystem());
    }

    public static Command LEDThreeFlashGreen(){
        return new RunCommand(() -> LEDStrip.getInstance().usePattern(new LEDPattern() {
            @Override
            public Color8Bit get(int led, double time) {
                return new Color8Bit(Color.fromHSV(121, 100, Math.sin(time/4)>0 ? 100 : 0));
            }
        }), SubsystemManager.getInstance().getLEDStripSubsystem()).withTimeout(Math.PI/4);
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

    public static Command cycleColors(double period, Color... colors){
        var singleColorLength = period/ colors.length;
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        Arrays.stream(colors).forEachOrdered(color -> {
            commandGroup.addCommands(LEDSolidColor(color).withTimeout(singleColorLength));
        });
        return commandGroup;
    }

    /*
        Intake coral - triple flash green -> solid green
        Honed -> triple flash green
        Not honed -> flash red
        Default disabled ->breathing alliance color
        Reef align -> flashing orange blue
        Default -> blue orange cycling
     */
}

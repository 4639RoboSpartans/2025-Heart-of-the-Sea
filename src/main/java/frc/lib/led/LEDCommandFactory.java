package frc.lib.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.util.DriverStationUtil;
import frc.robot.subsystems.SubsystemManager;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.IntStream;

public class LEDCommandFactory {
    static LEDStrip ledStripSubsystem = SubsystemManager.getInstance().getLEDStripSubsystem();
    public static Command LEDBreathingRed(){
        return new RunCommand(() -> ledStripSubsystem.usePattern(breathingRed), ledStripSubsystem).ignoringDisable(true);
    }

    public static Command LEDBreathingBlue(){
        return new RunCommand(() -> ledStripSubsystem.usePattern(breathingBlue), ledStripSubsystem).ignoringDisable(true);
    }

    public static Command LEDThreeFlashGreen(){
        return flashColors(0.3, Color.kGreen, Color.kBlack).ignoringDisable(true);
    }

    public static Command LEDFlashRed(){
        return flashColors(0.5, Color.kRed, Color.kBlack).ignoringDisable(true);
    }

    public static Command LEDSolidColor(Color color){
        return new RunCommand(() -> ledStripSubsystem.usePattern(new SolidLEDPattern(new Color8Bit(color))), ledStripSubsystem).ignoringDisable(true);
    }

    public static Command LEDThreeFlashThenSolidGreen(){
        return LEDThreeFlashGreen().andThen(LEDSolidColor(Color.kLimeGreen)).ignoringDisable(true);
    }

    public static Command flashColors(double period, Color... colors){
        var singleColorLength = period/ colors.length;
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        Arrays.stream(colors).forEachOrdered(color -> {
            commandGroup.addCommands(LEDSolidColor(color).withTimeout(singleColorLength));
        });
        return commandGroup.repeatedly().ignoringDisable(true);
    }

    public static Command flashBlueOrange(){
        return flashColors(1, Color.kBlue, Color.kOrange).ignoringDisable(true);
    }

    public static Command colorProfileBetween(Color startColor, Color endColor, double transitionLength){
        double[] rgbStart = new double[]{startColor.red * 255, startColor.green * 255, startColor.blue * 255};
        double[] rgbEnd = new double[]{endColor.red * 255, endColor.green * 255, endColor.blue * 255};

        double startTime = Timer.getFPGATimestamp();

        double[] differences = IntStream.range(0, 3).mapToDouble(i -> rgbEnd[i] - rgbStart[i]).toArray();

        return new RunCommand(() -> ledStripSubsystem.usePattern(new LEDPattern() {

            @Override
            public Color8Bit get(int led, double time) {
                return new Color8Bit(
                        (int) (rgbStart[0] + ((time-startTime)/transitionLength) * differences[0]),
                        (int) (rgbStart[1] + ((time-startTime)/transitionLength) * differences[1]),
                        (int) (rgbStart[2] + ((time-startTime)/transitionLength) * differences[2]));
            }
        }), ledStripSubsystem).until(() -> Timer.getFPGATimestamp() >= startTime + transitionLength).ignoringDisable(true);
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

        return commandGroup.repeatedly().ignoringDisable(true);
    }

    public static Command blueOrangeCycle() {
        return cycleBetweenColors(3, 0.5, Color.kBlue, Color.kOrange).ignoringDisable(true);
    }

    public static Command usePattern(LEDPattern pattern){
        return new RunCommand(() -> ledStripSubsystem.usePattern(pattern), ledStripSubsystem).ignoringDisable(true);
    }

    public static Command usePatternChooser(Function<LEDStrip, LEDPattern> chooser){
        return Commands.run(
                () -> ledStripSubsystem.usePattern(chooser.apply(ledStripSubsystem))
        , ledStripSubsystem).ignoringDisable(true);
    }

    public static Command disabledPatternChooser(){
        return usePatternChooser(_strip -> {
            if (RobotState.isDisabled() && DriverStationUtil.getAlliance() == DriverStation.Alliance.Red) return breathingRed;
            else return breathingBlue;
        });
    }


    public static LEDPattern breathingRed = new LEDPattern() {
        @Override
        public Color8Bit get(int led, double time) {
            return new Color8Bit((int) ((255-150) + 75 * Math.sin(time * 2)), 0, 0);
        }
    };

    public static LEDPattern breathingBlue = new LEDPattern() {
        @Override
        public Color8Bit get(int led, double time) {
            return new Color8Bit(0, 0, (int) ((255-150) + 75 * Math.sin(time * 2)));
        }
    };

    public static void setLEDCommand(Command command){
        if (command.getRequirements().contains(ledStripSubsystem)) CommandScheduler.getInstance().schedule(command);
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

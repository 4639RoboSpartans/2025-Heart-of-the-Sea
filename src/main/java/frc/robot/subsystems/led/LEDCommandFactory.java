package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.led.*;
import frc.lib.util.DriverStationUtil;
import frc.robot.subsystems.SubsystemManager;

public class LEDCommandFactory {
    static LEDStrip leds = SubsystemManager.getInstance().getLEDStripSubsystem();

    public static Command LEDBreathingRed() {
        return leds.usePattern(breathingRed);
    }

    public static Command LEDBreathingBlue() {
        return leds.usePattern(breathingBlue);
    }

    public static Command LEDThreeFlashGreen() {
        return leds.resetTime().andThen(
            leds.usePattern(new CycleBetweenLEDPattern(
                7, Color.kBlack, Color.kGreen
            )).withTimeout(1)
        );
    }

    public static Command LEDFlashRed() {
        return leds.usePattern(new CycleBetweenLEDPattern(
            2, Color.kRed, Color.kBlack
        ));
    }

    public static Command LEDThreeFlashGold(){
        return leds.resetTime().andThen(
            leds.usePattern(new CycleBetweenLEDPattern(
                7, Color.kBlack, Color.kGold
            )).withTimeout(1)
        );
    }

    public static Command LEDThreeFlashThenSolidGold(){
        return LEDThreeFlashGold().andThen(leds.usePattern(new SolidLEDPattern(Color.kGold)));
    }

    public static Command LEDThreeFlashThenSolidGreen() {
        return LEDThreeFlashGreen().andThen(leds.usePattern(new SolidLEDPattern(Color.kGreen)));
    }

    public static Command blueOrangeCycle() {
        return leds.usePattern(new FadeBetweenLEDPattern(2, Color.kOrange, Color.kBlue));
    }

    public static Command LEDFlashPurple() {
        return leds.usePattern(new CycleBetweenLEDPattern(3, Color.kPurple, Color.kBlack));
    }

    public static Command defaultCommand() {
        return leds.usePattern(() -> {
            if (RobotState.isDisabled()){
                if (DriverStationUtil.getAlliance() == DriverStation.Alliance.Red) return breathingRed;
                else return breathingBlue;
            } else return new SolidLEDPattern(Color.kBlack);
        });
    }

    public static LEDPattern breathingRed = new BreathingLEDPattern(
        Color.kRed, 0.5, .1, .7
    );

    public static LEDPattern breathingBlue = new BreathingLEDPattern(
        Color.kBlue, 0.5, .1, .7
    );

    public static void setLEDCommand(Command command) {
        if (command.getRequirements().contains(leds)) CommandScheduler.getInstance().schedule(command);
    }

    public static LEDPattern redOrangeMovingWave = new MovingWaveLEDPattern(
            Color.kRed, Color.kOrangeRed, 5
    );

    public static LEDPattern blueTealMovingWave = new MovingWaveLEDPattern(
            Color.kBlue, Color.kLimeGreen, 5
    );

    /*
        Intake coral - triple flash green -> solid green
        Honed -> triple flash green
        Not honed -> flash red
        Default disabled ->breathing alliance color
        Reef align -> flashing orange blue
        Default -> blue orange cycling
        Manual -> flash purple
     */
}

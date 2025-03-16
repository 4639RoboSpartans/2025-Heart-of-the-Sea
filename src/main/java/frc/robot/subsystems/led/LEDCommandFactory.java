package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.led.BreathingLEDPattern;
import frc.lib.led.CycleBetweenLEDPattern;
import frc.lib.led.LEDPattern;
import frc.lib.led.SolidLEDPattern;
import frc.lib.util.DriverStationUtil;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;
import frc.robot.subsystems.scoring.endeffector.AbstractEndEffectorSubsystem;

public class LEDCommandFactory {
    static LEDStrip leds = SubsystemManager.getInstance().getLEDStripSubsystem();

    private static LEDPattern disabledPattern() {
        return (led, time) -> switch (DriverStationUtil.getAlliance()) {
            case Red -> {
                double x = led * 0.1 + time * 3;
                double h = 8 * Math.pow(Math.sin(x / 2), 2);
                yield Color.fromHSV((int) h, 255, 255);
            }
            case Blue -> {
                time *= 2;
                double x = led * 0.2 + time * 3;
                double h = 20 * Math.pow(Math.sin(x), 2) + 90;
                double v = Math.pow(Math.sin(time), 2) * 0.9 + 0.1;

                yield Color.fromHSV((int) (h), 255, (int) (255 * v));
            }
        };
    }

    public static Command defaultCommand() {
        ScoringSuperstructure scoring = SubsystemManager.getInstance().getScoringSuperstructure();
        AbstractEndEffectorSubsystem endEffector = scoring.getEndEffectorSubsystem();
        AbstractSwerveDrivetrain drivetrain = SubsystemManager.getInstance().getDrivetrain();

        return leds.usePattern(() -> {
            // If disabled, use the disabled pattern (breathe red / blue)
            if (RobotState.isDisabled()) {
                return LEDCommandFactory.disabledPattern();
            }
            // If manual control enabled, blink purple
            if (scoring.isManualControlEnabled()) {
                return new CycleBetweenLEDPattern(7, Color.kPurple, Color.kBlack);
            }
            // If no coral, solid red
            if (!endEffector.hasCoral()) return (_1, _2) -> Color.kRed;

            // Figure out what colors to flash
            Color alignColor = Color.kYellow;
            Color scoringColor = new Color(1.0, 0.1, 0.0);

            boolean isAligning = drivetrain.isAligning();
            boolean isAligningFinished = isAligning && drivetrain.atTargetPose(drivetrain.currentAlignTarget);

            boolean isScoringExecuting = switch (scoring.getCurrentState()) {
                case EXECUTING_ACTION, DONE -> false;
                default -> true;
            };
            boolean isScoring = !(
                scoring.getCurrentAction() == ScoringSuperstructureAction.IDLE ||
                scoring.getCurrentAction() == ScoringSuperstructureAction.INTAKE_FROM_HP
            );
            boolean isPrepareScoringFinished = isScoring && isScoringExecuting;

            // If we aren't doing anything, use green
            if (!isAligning && !isScoring) {
                return new SolidLEDPattern(Color.kGreen);
            }

            LEDPattern scoringPattern = isPrepareScoringFinished
                ?
                new SolidLEDPattern(scoringColor)
                : isScoring
                ?
                new CycleBetweenLEDPattern(7,
                    scoringColor, isAligningFinished ? alignColor : Color.kBlack
                )
                :
                null;
            LEDPattern aligningPattern = isAligningFinished
                ?
                new SolidLEDPattern(alignColor)
                : isAligning
                ?
                new CycleBetweenLEDPattern(7,
                    alignColor, isPrepareScoringFinished ? scoringColor : Color.kBlack
                )
                :
                null;

            if(aligningPattern == null && scoringPattern == null) return new SolidLEDPattern(Color.kBlack);
            if(aligningPattern == null) return scoringPattern;
            if(scoringPattern == null) return aligningPattern;

            return (led, time) -> (led / 2) % 2 == 0 ? scoringPattern.get(led, time) : aligningPattern.get(led, time);
        }).ignoringDisable(true);
    }

    public static Command onCollectCoral() {
        return leds.resetTime().andThen(
            leds.usePattern(new CycleBetweenLEDPattern(
                7, Color.kBlack, Color.kGreen
            )).withTimeout(1)
        );
    }

//    public static LEDPattern breathingRed = new BreathingLEDPattern(
//        Color.kRed, 0.5, .1, .7
//    );
//
//    public static LEDPattern breathingBlue = new BreathingLEDPattern(
//        Color.kBlue, 0.5, .1, .7
//    );

    public static void setLEDCommand(Command command) {
        if (command.getRequirements().contains(leds)) CommandScheduler.getInstance().schedule(command);
    }

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

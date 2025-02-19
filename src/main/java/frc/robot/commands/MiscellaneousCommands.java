package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;

public class MiscellaneousCommands {

    /**
     * Jasper wants to run the elevator up and down 100 times so here we are
     */
    public static Command ElevatorUpDownTest(){
        Command c = Commands.repeatingSequence(
                SubsystemManager.getInstance().getScoringSuperstructure().setAction(ScoringSuperstructureAction.SCORE_BARGE),
                SubsystemManager.getInstance().getScoringSuperstructure().runScoringState().until(SubsystemManager.getInstance().getScoringSuperstructure().isAtPosition),
                SubsystemManager.getInstance().getScoringSuperstructure().setAction(ScoringSuperstructureAction.IDLE),
                SubsystemManager.getInstance().getScoringSuperstructure().runScoringState().until(SubsystemManager.getInstance().getScoringSuperstructure().isAtPosition)
        )
            // .onlyWhile(() -> OI.getInstance().driverController().A_BUTTON.getAsBoolean())
            //     .andThen(SubsystemManager.getInstance().getScoringSuperstructure().hold())
                ;

        c.addRequirements(SubsystemManager.getInstance().getScoringSuperstructure());
        return c;
    }
}

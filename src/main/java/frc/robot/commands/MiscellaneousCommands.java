package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

public class MiscellaneousCommands {

    /**
     * Jasper wants to run the elevator up and down 100 times so here we are
     */
    public Command ElevatorUpDownTest(){
        return Commands.repeatingSequence(
                SubsystemManager.getInstance().getScoringSuperstructure().setScoringState(ScoringSuperstructureState.BARGE_SCORING),
                SubsystemManager.getInstance().getScoringSuperstructure().runScoringState().until(SubsystemManager.getInstance().getScoringSuperstructure().isAtPosition),
                SubsystemManager.getInstance().getScoringSuperstructure().setScoringState(ScoringSuperstructureState.IDLE),
                SubsystemManager.getInstance().getScoringSuperstructure().runScoringState().until(SubsystemManager.getInstance().getScoringSuperstructure().isAtPosition)
        ).onlyWhile(() -> true /*TODO: put a button here*/)
                .andThen(SubsystemManager.getInstance().getScoringSuperstructure().hold());
    }

}

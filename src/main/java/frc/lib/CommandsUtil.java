package frc.lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;

/**
 * Utility class for creating and manipulating Commands. Wraps some functionality of {@link
 * Commands}.
 */
public class CommandsUtil {

    /**
     * Creates a new command that runs multiple commands sequentially. If only one Command is given,
     * then it will be returned unchanged (and not wrapped in another command).
     *
     * @param commands The commands to include in the group.
     * @return The command group.
     */
    public static Command sequence(List<Command> commands) {
        return sequence(commands.stream().toArray(Command[]::new));
    }

    /**
     * Creates a new command that runs multiple commands sequentially. If only one Command is given,
     * then it will be returned unchanged (and not wrapped in another command).
     *
     * @param commands The commands to include in the group.
     * @return The command group.
     */
    public static Command sequence(Command... commands) {
        if (commands.length == 1) return commands[0];

        return Commands.sequence(commands);
    }

    public static Command parallel(List<Command> commands) {
        return Commands.parallel(commands.stream().toArray(Command[]::new));
    }
}

package org.team7525.misc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

public class CommandsUtil {
	private static final Map<String, Integer> COMMAND_COUNTS = new HashMap<>();

	public static void logCommands() {
		BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
			String name = command.getName();
			int count = COMMAND_COUNTS.getOrDefault(name, 0) + (active ? 1 : -1);
			COMMAND_COUNTS.put(name, count);

			Logger.recordOutput(
					"CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()),
					active
			);
			Logger.recordOutput("CommandsAll/" + name, count > 0);
		};

		CommandScheduler.getInstance()
				.onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
		CommandScheduler.getInstance()
				.onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
		CommandScheduler.getInstance()
				.onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));
	}
}

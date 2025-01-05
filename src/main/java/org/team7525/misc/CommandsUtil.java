package org.team7525.misc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

public class CommandsUtil {

	// Taken from team 3256, ty for fixing our sysid
	public static void logCommands() {
		Map<String, Integer> commandCounts = new HashMap<>();
		BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
			String name = command.getName();
			int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
			commandCounts.put(name, count);

			Logger.recordOutput(
				"CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()),
				active
			);
			Logger.recordOutput("CommandsAll/" + name, count > 0);
		};
		CommandScheduler.getInstance()
			.onCommandInitialize((Command command) -> {
				logCommandFunction.accept(command, true);
			});
		CommandScheduler.getInstance()
			.onCommandFinish((Command command) -> {
				logCommandFunction.accept(command, false);
			});
		CommandScheduler.getInstance()
			.onCommandInterrupt((Command command) -> {
				logCommandFunction.accept(command, false);
			});
	}
}

package org.team7525.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.XboxController;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public abstract class Subsystem<StateType extends SubsystemStates> extends SubsystemBase {

	private Map<StateType, ArrayList<Trigger<StateType>>> triggerMap = new HashMap<>();
	private List<RunnableTrigger> runnableTriggerList = new ArrayList<>();

	private StateType state = null;
	private Timer stateTimer = new Timer();
	private String subsystemName;

	public static Supplier<XboxController> controllerSupplier = () -> new XboxController(99);
	private static boolean clearControllerCacheEachLoop = false;

	public Subsystem(String subsystemName, StateType defaultState) {
		if (defaultState == null) {
			throw new RuntimeException("Default state cannot be null!");
		}
		this.subsystemName = subsystemName;
		this.state = defaultState;

		stateTimer.start();
	}

	// State operation
	public void periodic() {
		// Commented out bc bad code
		// Logger.recordOutput(subsystemName + "/state", state.getStateString());
		// if (!DriverStation.isEnabled()) return;
		
		runState();

		checkTriggers();
		checkRunnableTriggers();
	}

	protected abstract void runState();

	// SmartDashboard utils
	protected void putSmartDashboard(String key, String value) {
		SmartDashboard.putString("[" + subsystemName + "] " + key, value);
	}

	protected void putSmartDashboard(String key, double value) {
		SmartDashboard.putNumber("[" + subsystemName + "] " + key, value);
	}

	// Triggers for state transitions
	protected void addTrigger(StateType startType, StateType endType, BooleanSupplier check) {
		if (triggerMap.get(startType) == null) {
			triggerMap.put(startType, new ArrayList<Trigger<StateType>>());
		}
		triggerMap.get(startType).add(new Trigger<StateType>(check, endType));
	}

	protected void addRunnableTrigger(Runnable runnable, BooleanSupplier check) {
		runnableTriggerList.add(new RunnableTrigger(check, runnable));
	}

	private void checkTriggers() {
		List<Trigger<StateType>> triggers = triggerMap.get(state);
		if (triggers == null) return;
		for (var trigger : triggers) {
			if (trigger.isTriggered()) {
				setState(trigger.getResultState());
				if (clearControllerCacheEachLoop) {
					clearControllerCache(controllerSupplier.get());
				}
				return;
			}
		}
	}

	private void checkRunnableTriggers() {
		if (runnableTriggerList == null) return;
		for (var trigger : runnableTriggerList) {
			if (trigger.isTriggered()) {
				trigger.run();
				return;
			}
		}
	}

	// Other utilities
	public StateType getState() {
		return state;
	}

	public void setState(StateType state) {
		if (this.state != state) stateTimer.reset();
		this.state = state;
	}

	/**
	 * Gets amount of time the state machine has been in the current state.
	 *
	 * @return time in seconds.
	 */
	protected double getStateTime() {
		return stateTimer.get();
	}

	public Command sysIdDynamic(Direction direction) {
		return new PrintCommand("Please Override Me!");
	}

	public Command sysIdQuasistatic(Direction direction) {
		return new PrintCommand("Please Override Me!");
	}

	public static void setControllerSupplier(Supplier<XboxController> supplier) {
		controllerSupplier = supplier;
	}

	public static void setClearControllerCacheEachLoop(boolean clearCache) {
		clearControllerCacheEachLoop = clearCache;
	}

	public static void clearControllerCache(XboxController controller) {
		// Call the function on the controller to clear the cache
		controller.getAButtonPressed();
		controller.getBButtonPressed();
		controller.getXButtonPressed();
		controller.getYButtonPressed();
		controller.getBackButtonPressed();
		controller.getLeftBumperButtonPressed();
		controller.getLeftStickButtonPressed();
		controller.getRightBumperButtonPressed();
		controller.getRightStickButtonPressed();
		controller.getStartButtonPressed();
	}
	
}

package org.team7525.subsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;

/**
 * A container for the state of a subsystem, prevents invalid states with eroneous vaues.
 * If your state is not included in one of the factory methods you need to split up your subsystem
 * or tell mech to build a different robot
 */
public record SubsystemState(
	String stateString,
	Measure<DistanceUnit> position,
	Rotation2d angularPosition,
	Measure<LinearVelocityUnit> velocity,
	Measure<AngularVelocityUnit> angularVelocity,
	Measure<AngularVelocityUnit> secondaryVelocity,
	Measure<LinearAccelerationUnit> acceleration
) {
	/**
	 * Primary constructor for all fields.
	 * Validates the state to prevent invalid combinations.
	 */
	public SubsystemState {
		validateState();
	}

	/**
	 * Static factory method for defining a subsystem with pivot states
	 */
	public static SubsystemState fromPivotStates(String stateString, Rotation2d angularPosition) {
		return new SubsystemState(stateString, null, angularPosition, null, null, null, null);
	}

	/**
	 * Static factory method for defining a subsystem with just elevator states
	 */
	public static SubsystemState fromElevatorStates(
		String stateString,
		Measure<DistanceUnit> position,
		Measure<LinearVelocityUnit> velocity
	) {
		return new SubsystemState(stateString, position, null, velocity, null, null, null);
	}

	/**
	 * Static factory method for defining a subsystem with just intake states
	 */
	public static SubsystemState fromOpeningIntakeStates(
		String stateString,
		Rotation2d angularPosition,
		Measure<AngularVelocityUnit> angularVelocity
	) {
		return new SubsystemState(
			stateString,
			null,
			angularPosition,
			null,
			angularVelocity,
			null,
			null
		);
	}

	/**
	 * Static factory method for defining a subsystem with just intake states
	 */
	public static SubsystemState fromIntakeStates(
		String stateString,
		Measure<AngularVelocityUnit> angularVelocity
	) {
		return new SubsystemState(stateString, null, null, null, angularVelocity, null, null);
	}

	/**
	 * Static factory method for defining a subsystem with just flywheel states
	 */
	public static SubsystemState fromFlywheelStates(
		String stateString,
		Measure<AngularVelocityUnit> velocity
	) {
		return new SubsystemState(stateString, null, null, null, velocity, null, null);
	}

	/**
	 * Static factory method for defining a subsystem with just flywheel states (uneven flywheels for spin)
	 */
	public static SubsystemState fromUnevenFlywheelStates(
		String stateString,
		Measure<AngularVelocityUnit> velocity,
		Measure<AngularVelocityUnit> secondaryVelocity
	) {
		return new SubsystemState(stateString, null, null, null, velocity, secondaryVelocity, null);
	}

	/**
	 * Static factory method for defining a subsystem with spinner and pivot states
	 */
	public static SubsystemState fromPivotWithSpinnerStates(
		String stateString,
		Rotation2d angularPosition,
		Measure<AngularVelocityUnit> angularVelocity
	) {
		return new SubsystemState(
			stateString,
			null,
			angularPosition,
			null,
			angularVelocity,
			null,
			null
		);
	}

	/**
	 * Static factory method for defining a subsystem with just turret states (mech pls build this)
	 */
	public static SubsystemState fromTurettStates(
		String stateString,
		Rotation2d angularPosition,
		Measure<AngularVelocityUnit> angularVelocity
	) {
		return new SubsystemState(
			stateString,
			null,
			angularPosition,
			null,
			angularVelocity,
			null,
			null
		);
	}

	/**
	 * No stupid states!!!
	 */
	private void validateState() {
		if (position != null && position.in(Meters) > 10) {
			throw new IllegalArgumentException(
				"U prob have conversion errors or L code (position > 10m)"
			);
		}

		if (velocity != null && velocity.in(MetersPerSecond) > 40) {
			throw new IllegalArgumentException(
				"U prob have conversion errors or L code (velocity > 40m/s)"
			);
		}

		if (
			(angularVelocity != null && angularVelocity.in(RotationsPerSecond) > 200) ||
			(secondaryVelocity != null && secondaryVelocity.in(RPM) / 60 > 200)
		) {
			throw new IllegalArgumentException(
				"What motor is going 200 RPS or why is gearing so low? fix ur robot or code"
			);
		}

		if (acceleration != null && acceleration.in(MetersPerSecondPerSecond) > 30) {
			throw new IllegalArgumentException(
				"U prob have conversion errors or L code (acceleration > 30 m/s^2)"
			);
		}
	}
}

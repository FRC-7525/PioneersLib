package org.team7525.controlConstants;

/**
 * The FFConstants class holds the feedforward constants used in control algorithms.
 * These constants are typically used to calculate the necessary input to achieve a desired output.
 * 
 * <p>Constants:</p>
 * <ul>
 *   <li>kS - Static friction constant</li>
 *   <li>kV - Velocity constant</li>
 *   <li>kA - Acceleration constant</li>
 *   <li>kG - Gravity constant</li>
 * </ul>
 * 
 * <p>Constructors:</p>
 * <ul>
 *   <li>{@link #FFConstants(double, double, double, double)} - Initializes all constants.</li>
 *   <li>{@link #FFConstants(double, double, double)} - Initializes kS, kV, and kA, with kG set to 0.</li>
 *   <li>{@link #FFConstants(double, double)} - Initializes kS and kV, with kA and kG set to 0.</li>
 *   <li>{@link #FFConstants(double)} - Initializes kV, with kS, kA, and kG set to 0.</li>
 * </ul>
 */
public class FFConstants {

	public double kS;
	public double kV;
	public double kA;
	public double kG;

	public FFConstants(double ks, double kg, double kv, double ka) {
		this.kS = ks;
		this.kV = kv;
		this.kA = ka;
		this.kG = kg;
	}

	public FFConstants(double ks, double kv, double ka) {
		this(ks, 0, kv, ka);
	}

	public FFConstants(double ks, double kv) {
		this(ks, 0, kv, 0);
	}

	public FFConstants(double kv) {
		this(0, 0, kv, 0);
	}
}

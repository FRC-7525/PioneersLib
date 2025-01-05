package org.team7525.misc;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;

public class VisionUtil {

	// Made by team 6443 AEMBOT
	// Camera Quality
	public enum CameraResolution {
		HIGH_RES,
		NORMAL,
	}

	// Made by team 6443 AEMBOT
	private static final Matrix<N3, N1> highResSingleTagStdDev = VecBuilder.fill(
		0.4,
		0.4,
		Double.MAX_VALUE
	);
	private static final Matrix<N3, N1> normalSingleTagStdDev = VecBuilder.fill(
		0.8,
		0.8,
		Double.MAX_VALUE
	);
	private static final Matrix<N3, N1> highResMultiTagStdDev = VecBuilder.fill(0.2, 0.2, 3);
	private static final Matrix<N3, N1> normalMultiTagStdDev = VecBuilder.fill(
		0.5,
		0.5,
		Double.MAX_VALUE
	);

	private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
		AprilTagFields.k2024Crescendo
	);

	// Made by team 6443 AEMBOT (peak PNW team)
	/**
	 * The standard deviations of the estimated poses from vision cameras, for use
	 * with {@link
	 * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
	 * SwerveDrivePoseEstimator}.
	 *
	 * @param estimatedPose The estimated pose to guess standard deviations for.
	 */
	public static Matrix<N3, N1> getEstimationStdDevs(
		EstimatedRobotPose estimatedPose,
		CameraResolution resolution
	) {
		var estStdDevs =
			switch (resolution) {
				case HIGH_RES -> highResSingleTagStdDev;
				case NORMAL -> normalSingleTagStdDev;
			};
		var targets = estimatedPose.targetsUsed;
		int numTags = 0;
		double avgDist = 0;
		for (var tgt : targets) {
			var tagPose = fieldLayout.getTagPose(tgt.getFiducialId());
			if (tagPose.isEmpty()) continue;
			numTags++;
			avgDist += tagPose
				.get()
				.toPose2d()
				.minus(estimatedPose.estimatedPose.toPose2d())
				.getTranslation()
				.getNorm();
		}

		if (numTags == 0) return estStdDevs;
		avgDist /= numTags;

		// Decrease std devs if multiple targets are visible
		if (
			numTags > 1 &&
			avgDist >
			switch (resolution) {
				case HIGH_RES -> 8;
				case NORMAL -> 5;
			}
		) {
			estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
		} else {
			estStdDevs = switch (resolution) {
				case HIGH_RES -> highResMultiTagStdDev;
				case NORMAL -> normalMultiTagStdDev;
			};
		}
		// Increase std devs based on (average) distance
		if (
			numTags == 1 &&
			avgDist >
			switch (resolution) {
				case HIGH_RES -> 6;
				case NORMAL -> 4;
			}
		) {
			estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
		} else {
			estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist) / 20));
		}

		return estStdDevs;
	}
}

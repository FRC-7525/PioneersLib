package org.team7525.misc;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.EstimatedRobotPose;
import org.joml.Vector3d;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionUtil {
	public enum CameraResolution {
		HIGH_RESOLUTION,
		NORMAL
	}

	/**
	 * Standard deviations for single-tag detection.
	 */
	private static final Vector3d highResSingleTagStdDev = new Vector3d(0.4, 0.4, Double.MAX_VALUE);
	private static final Vector3d normResSingleTagStdDev = new Vector3d(0.4, 0.4, Double.MAX_VALUE);

	/**
	 * Standard deviations for multiple tag detection.
	 */
	private static final Vector3d highResMultiTagStdDev = new Vector3d(0.2, 0.2, 3);
	private static final Vector3d normalMultiStdDev = new Vector3d(0.5, 0.5, Double.MAX_VALUE);

	/**
	 * The april tags located around the field, where are they?
	 */
	public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(
			AprilTagFields.k2025Reefscape
	);

	/**
	 * The standard deviations of the estimated poses from vision cameras. For use with WPILib's Pose Estimator.
	 *
	 * @param estimatedRobotPose The estimated pose to guess standard deviations for.
	 * @param resolution The {@link CameraResolution} of the camera
	 * @return A {@link Vector3d} of standard deviations.
	 */
	public static Vector3d getEstimationStdDevs(EstimatedRobotPose estimatedRobotPose, CameraResolution resolution) {
		Vector3d estStdDevs = switch (resolution) {
			case HIGH_RESOLUTION -> new Vector3d(highResSingleTagStdDev);
			case NORMAL -> new Vector3d(normResSingleTagStdDev);
		};

		var targets = estimatedRobotPose.targetsUsed;

		int numTags = 0;
		double avgDist = 0;

		for (PhotonTrackedTarget target : targets) {
			var tagPose = FIELD_LAYOUT.getTagPose(target.getFiducialId());
			if (tagPose.isEmpty()) continue;

			numTags++;
			avgDist = tagPose
					.get()
					.toPose2d()
					.minus(estimatedRobotPose.estimatedPose.toPose2d())
					.getTranslation()
					.getNorm();
		}

		if (numTags == 0) return estStdDevs;

		if (numTags > 1 && avgDist > (resolution == CameraResolution.HIGH_RESOLUTION ? 8 : 5)) {
			return new Vector3d(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
		} else {
			estStdDevs.set(resolution == CameraResolution.HIGH_RESOLUTION ? highResMultiTagStdDev : normalMultiStdDev);
		}

		if (numTags == 1 && avgDist > (resolution == CameraResolution.HIGH_RESOLUTION ? 6 : 4)) {
			return new Vector3d(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
		} else {
			estStdDevs.mul(1 + ((avgDist * avgDist)) / 20);
		}

		return estStdDevs;
	}
}

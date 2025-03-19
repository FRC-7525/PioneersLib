package org.team7525.misc;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

public final class Elastic {

	private static final StringTopic topic = NetworkTableInstance.getDefault()
			.getStringTopic("/Elastic/RobotNotifications");
	private static final StringPublisher publisher = topic.publish(
			PubSubOption.sendAll(true),
			PubSubOption.keepDuplicates(true)
	);
	private static final ObjectMapper objectMapper = new ObjectMapper();

	/**
	 * Sends an alert notification to the Elastic dashboard.
	 * The alert is serialized as a JSON string before being published.
	 *
	 * @param alert the {@link ElasticNotification} object containing alert details
	 */
	public static void sendAlert(ElasticNotification alert) {
		try {
			String alertJson = objectMapper.writeValueAsString(alert);
			publisher.set(alertJson); // Avoiding multiple string allocations
		} catch (JsonProcessingException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Represents a notification object to be sent to the Elastic dashboard.
	 * This object holds properties such as level, title, description, display time,
	 * and dimensions to control how the alert is displayed on the dashboard.
	 */
	public static class ElasticNotification {

		@JsonProperty("level")
		private final NotificationLevel level;

		@JsonProperty("title")
		private final String title;

		@JsonProperty("description")
		private final String description;

		@JsonProperty("displayTime")
		private final int displayTimeMillis;

		@JsonProperty("width")
		private final double width;

		@JsonProperty("height")
		private final double height;

		/**
		 * Creates a new ElasticNotification with all properties specified.
		 *
		 * @param level             the level of the notification (e.g., INFO, WARNING, ERROR)
		 * @param title             the title text of the notification
		 * @param description       the descriptive text of the notification
		 * @param displayTimeMillis the time in milliseconds for which the notification is displayed
		 * @param width             the width of the notification display area
		 * @param height            the height of the notification display area, inferred if below zero
		 */
		public ElasticNotification(
				NotificationLevel level,
				String title,
				String description,
				int displayTimeMillis,
				double width,
				double height
		) {
			this.level = level;
			this.title = title;
			this.description = description;
			this.displayTimeMillis = displayTimeMillis;
			this.width = width;
			this.height = height < 0 ? (width * 0.5) : height; // Inference logic moved here
		}

		/**
		 * Creates a new ElasticNotification with default display time and dimensions.
		 *
		 * @param level       the level of the notification
		 * @param title       the title text of the notification
		 * @param description the descriptive text of the notification
		 */
		public ElasticNotification(NotificationLevel level, String title, String description) {
			this(level, title, description, 3000, 350, -1);
		}

		/**
		 * Creates a new ElasticNotification with a specified display time and default dimensions.
		 *
		 * @param level             the level of the notification
		 * @param title             the title text of the notification
		 * @param description       the descriptive text of the notification
		 * @param displayTimeMillis the display time in milliseconds
		 */
		public ElasticNotification(
				NotificationLevel level,
				String title,
				String description,
				int displayTimeMillis
		) {
			this(level, title, description, displayTimeMillis, 350, -1);
		}

		public NotificationLevel getLevel() {
			return level;
		}

		public String getTitle() {
			return title;
		}

		public String getDescription() {
			return description;
		}

		public int getDisplayTimeMillis() {
			return displayTimeMillis;
		}

		public double getWidth() {
			return width;
		}

		public double getHeight() {
			return height;
		}

		/**
		 * Represents the possible levels of notifications for the Elastic dashboard.
		 * These levels are used to indicate the severity or type of notification.
		 */
		public enum NotificationLevel {
			INFO, WARNING, ERROR
		}
	}
}

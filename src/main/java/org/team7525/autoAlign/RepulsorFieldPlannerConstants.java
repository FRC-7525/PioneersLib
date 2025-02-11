package org.team7525.autoAlign;

import java.util.List;

import org.team7525.autoAlign.RepulsorFieldPlanner.GuidedObstacle;
import org.team7525.autoAlign.RepulsorFieldPlanner.HorizontalObstacle;
import org.team7525.autoAlign.RepulsorFieldPlanner.Obstacle;
import org.team7525.autoAlign.RepulsorFieldPlanner.VerticalObstacle;

import edu.wpi.first.math.geometry.Translation2d;


/*
 * EVERYTHING IS IN METERS AND METRIC UNITS
 */
public final class RepulsorFieldPlannerConstants {
    public static final double GOAL_STRENGTH = 0.65;
	static final double FIELD_LENGTH = 16.42;
	static final double FIELD_WIDTH = 8.16;

    public static final class DefaultObstalces {
        public static final List<Obstacle> FIELD_OBSTACLES = List.of(
		    new GuidedObstacle(new Translation2d(4.49, 4), 1, true),
		    new GuidedObstacle(new Translation2d(13.08, 4), 1, true)
	    );

	    public static final List<Obstacle> WALLS = List.of(
	    	new HorizontalObstacle(0.0, 0.5, true),
	    	new HorizontalObstacle(FIELD_WIDTH, 0.5, false),
	    	new VerticalObstacle(0.0, 0.5, true),
	    	new VerticalObstacle(FIELD_LENGTH, 0.5, false),
	    	new VerticalObstacle(7.55, 0.5, false),
	    	new VerticalObstacle(10, 0.5, true)
	    );
    }
}

package org.team7525.autoAlign;

import java.lang.reflect.Field;
import java.util.List;
import static org.team7525.autoAlign.RepulsorFieldPlanner.Obstacle;
import static org.team7525.autoAlign.RepulsorFieldPlanner.PointObstacle;
import static org.team7525.autoAlign.RepulsorFieldPlanner.GuidedObstacle;
import static org.team7525.autoAlign.RepulsorFieldPlanner.HorizontalObstacle;
import static org.team7525.autoAlign.RepulsorFieldPlanner.VerticalObstacle;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;


/*
 * EVERYTHING IS IN METERS AND METRIC UNITS
 */
public final class RepulsorFieldPlannerConstants {
    public static final double GOAL_STRENGTH = 0.65;
	public static int ARROWS_ON_X_AXIS = 40;
	public static int ARROWS_ON_Y_AXIS = 20;
	public static final double FIELD_LENGTH = 16.42;
	public static final double FIELD_WIDTH = 8.16;	

    public static final class DefaultObstalces {
        public static final List<Obstacle> FIELD_OBSTACLES = List.of(
			new GuidedObstacle(1.0, true, Meters.of(0.5), new Translation2d(4.49, 4)),
			new GuidedObstacle(1.0, true, Meters.of(0.5), new Translation2d(13.08, 4))
	    );

	    public static final List<Obstacle> WALLS = List.of(
			new HorizontalObstacle(0.5, true, 0, Meters.of(1)),
			new HorizontalObstacle(0.5, true, FIELD_WIDTH, Meters.of(1)),
			new VerticalObstacle(0.5, true, 0, Meters.of(1)),
			new VerticalObstacle(0.5, true, FIELD_LENGTH, Meters.of(1)),
			new VerticalObstacle(0.5, false, 7.55, Meters.of(1)),
			new VerticalObstacle(0.5, false, 10, Meters.of(1))
	    );
    }
}

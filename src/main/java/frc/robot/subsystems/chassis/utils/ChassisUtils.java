package frc.robot.subsystems.chassis.utils;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.utils.UtilsGeneral;

public class ChassisUtils {
    /**
     * The zone the robot is in
     */
    public static enum Zone {
        LOADING_ZONE, OPEN_AREA, COMMUNITY_TOP, COMMUNITY_MIDDLE, COMMUNITY_BOTTOM, COMMUNITY_ENTRANCE_TOP,
        COMMUNITY_ENTRANCE_BOTTOM;

        /**
         * Gets the zone the robot is in from its position
         * 
         * @param robotPosition The robot's position
         * @return The zone the robot is in
         */
        public static Zone fromRobotLocation(Translation2d robotPosition) {
            if (UtilsGeneral.isRedAlliance())
                robotPosition = new Translation2d(Constants.FIELD_WIDTH - robotPosition.getX(), robotPosition.getY());
            if (Constants.COMMUNITY_BOTTOM.isInside(robotPosition))
                return COMMUNITY_BOTTOM;
            if (Constants.COMMUNITY_MIDDLE.isInside(robotPosition))
                return COMMUNITY_MIDDLE;
            if (Constants.COMMUNITY_TOP.isInside(robotPosition))
                return COMMUNITY_TOP;
            if (Constants.ENTRANCE_BOTTOM.isInside(robotPosition))
                return COMMUNITY_ENTRANCE_BOTTOM;
            if (Constants.ENTRANCE_TOP.isInside(robotPosition))
                return COMMUNITY_ENTRANCE_TOP;
            if (Constants.LOADING_ZONE.isInside(robotPosition))
                return LOADING_ZONE;
            return OPEN_AREA;
        }
    }

    /**
     * Creates a path point with the position and heading relative to the alliance
     * 
     * @param position          The position of the point
     * @param heading           The heading of the point
     * @param holonomicRotation The holonomic rotation of the point
     * @param velocity          The velocity of the point, -1 for default
     * @param alliance          The alliance the point is relative to
     * @return The path point, with the position and heading relative to the
     *         alliance
     */
    public static PathPoint createAllianceRelativePathPoint(Translation2d position, Rotation2d heading,
            Rotation2d holonomicRotation, double velocity, Alliance alliance) {

        if (DriverStation.getAlliance() != alliance) {
            position = new Translation2d(Constants.FIELD_WIDTH - position.getX(), position.getY());
            heading = heading.rotateBy(Rotation2d.fromDegrees(180));
            holonomicRotation = holonomicRotation.rotateBy(Rotation2d.fromDegrees(180));
        }
        return new PathPoint(position, heading, holonomicRotation, velocity);
    }
}

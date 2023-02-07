package frc.robot.subsystems.parallelogram;

import edu.wpi.first.math.geometry.Rotation2d;

public class Utils {

    /**
     * Calculates the wanted parallelogram angle using the desired height.
     * 
     * @param height the height of the wanted node.
     * @param isFront is the wanted arm direction front or left.
     * @return Returns the paralellogram angle.
     */
    public static double calculateAngle(double height, boolean isFront) {
        double angle = Math.asin((height - ParallelConstants.ROBOT_HEIGHT) /
                ParallelConstants.PARALLEL_LENGTH);
        
        if (!isFront) {
            angle = 2*Math.PI - angle;
        }
        return toDegrees(angle);
    }

    public static double toDegrees(double rads) {
        return new Rotation2d(rads).getDegrees();
    }
}

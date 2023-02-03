package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.Constants.ParallelConstants;

public class ParallelogramUtils {

    /**
     * Calculates the wanted parallelogram angle using the desired height.
     * 
     * @param height The height of the wanted node.
     * @return Returns the paralellogram angle.
     */
    public static double calculateAngle(double height) {
        double angle = Math.asin((height - Constants.ParallelConstants.ROBOT_HEIGHT) /
                Constants.ParallelConstants.PARALLEL_LENGTH);
        return angle;
    }
}

package frc.robot.subsystems.parallelogram;


public class Utils {

    /**
     * Calculates the wanted parallelogram angle using the desired height.
     * 
     * @param height The height of the wanted node.
     * @return Returns the paralellogram angle.
     */
    public static double calculateAngle(double height) {
        double angle = Math.asin((height - ParallelConstants.ROBOT_HEIGHT) /
                ParallelConstants.PARALLEL_LENGTH);
        return angle;
    }
}

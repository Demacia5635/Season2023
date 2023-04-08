package frc.robot.subsystems.parallelogram;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

public class ParallelogramUtils {

    /**
     * Calculates the wanted parallelogram angle using the desired height.
     * 
     * @param height  the height of the wanted node.
     * @param isFront is the wanted arm direction front or left.
     * @return Returns the paralellogram angle.
     */
    public static double calculateAngle(double height, boolean isFront) {
        double angle = Math.asin((height - ParallelConstants.ROBOT_HEIGHT) /
                ParallelConstants.PARALLEL_LENGTH);

        if (!isFront) {
            angle = 2 * Math.PI - angle;
        }

        return toDegrees(angle);
    }

    /**
     * Converts angle from radians to degrees.
     * 
     * @param rads the angle in radians
     * @return the angle in degrees.
     */
    public static double toDegrees(double rads) {
        return new Rotation2d(rads).getDegrees();
    }

    /**
     * Converts angle from degrees to radians.
     * 
     * @param degrees the angle in degrees
     * @return the angle in radians
     */
    public static double toRads(double degrees) {
        return Rotation2d.fromDegrees(degrees).getRadians();
    }

    /**
     * Checks if the a button on the controller was pressed.
     * 
     * @param controller The xbox controller to get the valus from.
     * @return if the a button was pressed
     */
    public static boolean aButtonPressed(XboxController controller) {
        return controller.getAButtonPressed();
    }

    /**
     * Checks if the b button on the controller was pressed.
     * 
     * @param controller The xbox controller to get the valus from.
     * @return if the b button was pressed
     */
    public static boolean bButtonPressed(XboxController controller) {
        return controller.getBButtonPressed();
    }

    // public static double calculatePower(double currentAngle, double setPointAngle) {
    //     if ((setPointAngle<90) && (currentAngle>90)) {
            
    //     }
    // }
}

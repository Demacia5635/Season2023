package frc.robot.utils;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

/**
 * Utility class for vision
 */
public class VisionUtils {

    /**
     * Sets up a listener for the vision system
     * 
     * @param listener The listener to call when a new pose is received
     */
    public static void setupVisionListener(Consumer<Pair<Pose2d, Double>> listener) {
        NetworkTableInstance.getDefault().addListener(VisionConstants.LIMELIGHT_TABLE.getEntry("botpose"),
                EnumSet.of(Kind.kValueAll), (event) -> {
                    Pair<Pose2d, Double> pose = getVisionPose();
                    if (pose != null)
                        listener.accept(pose);
                });
    }

    /**
     * Gets the pose of the robot from the vision system
     * 
     * @return The pose of the robot from the vision system, and the timestamp of
     *         the measurement, or null if no target is found
     */
    public static Pair<Pose2d, Double> getVisionPose() {
        double hasTarget = VisionConstants.LIMELIGHT_TABLE.getEntry("tv").getDouble(0);
        if (hasTarget == 0)
            return null;

        double[] robotPose = VisionConstants.LIMELIGHT_TABLE.getEntry("botpose").getDoubleArray(new double[0]);
        if (robotPose.length != 6)
            return null;

        double latency = VisionConstants.LIMELIGHT_TABLE.getEntry("tl").getDouble(0);
        robotPose[0] = Constants.FIELD_WIDTH / 2 + robotPose[0];
        robotPose[1] = robotPose[1] + Constants.FIELD_HEIGHT / 2;

        Rotation2d robotRotation = Rotation2d.fromDegrees(robotPose[5]);
        Translation2d robotTranslation = new Translation2d(robotPose[0], robotPose[1]);

        return new Pair<Pose2d, Double>(
                new Pose2d(robotTranslation, robotRotation),
                Timer.getFPGATimestamp() - ((latency + VisionConstants.CAPTURE_LATENCY) / 1000));
    }
}

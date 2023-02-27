package frc.robot.utils;

import java.util.ConcurrentModificationException;
import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        NetworkTableInstance.getDefault().addListener(VisionConstants.ROBOT_POSE_ENTRY,
                EnumSet.of(Kind.kValueAll), (event) -> {
                    Pair<Pose2d, Double> pose = getVisionPose(VisionConstants.LIMELIGHT_TABLE1);
                    if (pose != null) {
                        try {
                            listener.accept(pose);
                        } catch (ConcurrentModificationException e) {
                            System.out.println("ConcurrentModificationException");
                        }
                    }
                });
        // NetworkTableInstance.getDefault().addListener(VisionConstants.LIMELIGHT_TABLE2.getEntry("botpose_wpiblue"),
        //         EnumSet.of(Kind.kValueAll), (event) -> {
        //             Pair<Pose2d, Double> pose = getVisionPose(VisionConstants.LIMELIGHT_TABLE2);
        //             if (pose != null) {
        //                 try {
        //                     listener.accept(pose);
        //                 } catch (ConcurrentModificationException e) {
        //                     System.out.println("ConcurrentModificationException");
        //                 }
        //             }
        //         });
    }

    /**
     * Gets the pose of the robot from the vision system
     * 
     * @return The pose of the robot from the vision system, and the timestamp of
     *         the measurement, or null if no target is found
     */
    public static Pair<Pose2d, Double> getVisionPose(NetworkTable limelightTable) {
        double hasTarget = limelightTable.getEntry("tv").getDouble(0);
        if (hasTarget == 0)
            return null;

        double[] robotPose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        if (robotPose.length != 7)
            return null;

        double latency = robotPose[6];

        Rotation2d robotRotation = Rotation2d.fromDegrees(robotPose[5]);
        Translation2d robotTranslation = new Translation2d(robotPose[0], robotPose[1]);

        double[] camTran = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
        double distance = Math.hypot(camTran[0], camTran[2]);
        SmartDashboard.putNumber("OurLimeLight/Distance", distance);
        SmartDashboard.putNumber("OurLimeLight/X", robotTranslation.getX());
        SmartDashboard.putNumber("OurLimeLight/Y", robotTranslation.getY());
        if (distance > VisionConstants.MAX_DISTANCE_FOR_LIMELIGHT)
            return null;

        return new Pair<Pose2d, Double>(
                new Pose2d(robotTranslation, robotRotation),
                Timer.getFPGATimestamp() - (latency / 1000));
    }
}

package frc.robot.subsystems.chassis;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Swerve Drive constants.
 */
public class ChassisConstants {
    /**
     * The Swerve Modules constants.
     */
    public static class SwerveModuleConstants {
        public final double angleOffset;
        public final int moveMotorID;
        public final int angleMotorID;
        public final int absoluteEncoderID;

        public static final double VELOCITY_KP = 2e-5;
        public static final double VELOCITY_KI = 6e-5;
        public static final double VELOCITY_KS = 0.0128;
        public static final double VELOCITY_KV = 0.227675;
        public static final SimpleMotorFeedforward VELOCITY_FF = new SimpleMotorFeedforward(VELOCITY_KS, VELOCITY_KV);
        public static final double ANGLE_KP = 0.2;
        public static final double ANGLE_KI = 0.002;
        public static final double ANGLE_KD = 5;
        public static final double MAX_ACCUM_INTEGRAL = 80000;
        public static final double MAX_CURRENT_ANGLE = 20;
        
        public static final double ANGLE_KF = 0;
        public static final double MAX_VELOCITY_ANGLE = 20000;
        public static final double MAX_ACCELERATION_ANGLE = 15000;
        public static final int S_CURVE_STRENGTH_ANGLE = 1;
  
        public static final double PPR_FALCON = 2048;
        public static final double WHEEL_PERIMITER = 0.1016 * Math.PI; // meters
        public static final double GEAR_RATIO_VEL = 8.14;
        public static final double PULSE_PER_METER = PPR_FALCON * GEAR_RATIO_VEL / WHEEL_PERIMITER;

        public static final double GEAR_RATIO_ANGLE = 12.8;
        public static final double PULSE_PER_DEGREE = PPR_FALCON * GEAR_RATIO_ANGLE / 360;

        /**
         * Creates a new SwerveModuleConstants.
         * 
         * @param angleOffset       The offset of the absolute encoder from the module
         * @param moveMotorID       The CAN ID of the drive motor
         * @param angleMotorID      The CAN ID of the angle motor
         * @param absoluteEncoderID The CAN ID of the absolute encoder
         */
        private SwerveModuleConstants(double angleOffset, int moveMotorID, int angleMotorID, int absoluteEncoderID) {
            this.angleOffset = angleOffset;
            this.moveMotorID = moveMotorID;
            this.angleMotorID = angleMotorID;
            this.absoluteEncoderID = absoluteEncoderID;
        }

        public static final SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants(209.443359375, 7, 8, 11);
        public static final SwerveModuleConstants FRONT_RIGHT = new SwerveModuleConstants(54.580078125, 5, 6, 13);
        public static final SwerveModuleConstants BACK_LEFT = new SwerveModuleConstants(189.755859375, 1, 2, 10);
        public static final SwerveModuleConstants BACK_RIGHT = new SwerveModuleConstants(218.49609375, 3, 4, 12);
    }

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(0.26515, 0.2215), // front left
            new Translation2d(0.26515, -0.2215), // front right
            new Translation2d(-0.25451, 0.2065), // back left - changed values because the chassis is curved. prev values were: -0.26515, 0.2215, change values are -0.25451, 0.2065
            new Translation2d(-0.26515, -0.2215) // back right
    );

    public static final int GYRO_ID = 14;

    public static final double MAX_SPEED = 4.2; // meters per second
    public static final double MAX_ACCELERATION = 8; // meters per second squared
    public static final double MAX_AUTO_ACCELERATION = 3; // meters per second squared
    public static final double MAX_AUTO_SPEED = 2;
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(MAX_AUTO_SPEED, MAX_AUTO_ACCELERATION);
    public static final PathConstraints PATH_CONSTANTS_AUTO_MIDDLE = new PathConstraints(0.75, MAX_AUTO_ACCELERATION);
    public static final double MAX_DRIVE_SPEED = 4;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    public static final double AUTO_TRANSLATION_KP = 2;
    public static final double AUTO_TRANSLATION_KI = 0.4;
    public static final double AUTO_ROTATION_KP = 1.35;
    public static final double AUTO_ROTATION_KI = 0.8;

    public static final double TELEOP_ROTATION_KP = 3;
    public static final double TELEOP_ROTATION_KI = 0.3;

    public static final double TELEOP_ANGLE_TOLERANCE = Math.toRadians(1.5);

    public static final double AUTO_ANGLE_TOLERANCE = Math.toRadians(2);
    public static final double AUTO_TRANSLATION_TOLERANCE = 0.02;

    public static final double STOP_VELOCITY = 0.3;
}

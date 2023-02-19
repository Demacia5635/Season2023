package frc.robot.subsystems.parallelogram;

/**
 * Parallelogram constants.
 */
public final class ParallelConstants {
    public static final int PORT_NUMBER_PARALLEL_MOTOR = 30;
    public static final boolean MOTOR_INVERT_TYPE = true;
    public static final double PARALLEL_LENGTH = 0.7; // In meters
    public static final double ROBOT_HEIGHT = 0.61; // 0.58

    public static final int GYRO_PORT_NUMBER = -4;
    public static final double GYRO_OFFSET = 0;

    public static final double HEIGHT_ANGLE_ERROR = 14.47;
    public static final double TRANSMITION_RATIO_ERROR = 1.1275;
    public static final double PULSE_PER_ANGLE = 1024 / TRANSMITION_RATIO_ERROR;

    public static final double PULI_RADIUS = 1;
    public static final double PULI_PERIMETER = 2 * PULI_RADIUS * Math.PI;

    public static final double KS_VELOCITY = 0;
    public static final double KV_VELOCITY = 0;

    public static final double DIGITAL_INPUT_ANGLE = 129.1;
    public static final int PORT_DIGITAL_INPUT = 0;

    public static final double KP_POSITION = 0.043;
    public static final double KI_POSITION = 0;
    public static final double KD_POSITION = 0;

    public static final double KP_VELOCITY = 0;
    public static final double KI_VELOCITY = 0;
    public static final double KD_VELOCITY = 0;

    public static final double CALIBRATION_POWER = 0.3;
    public static final double BACKWARDS_CALIBRATION_POWER = -0.15;

    public static final double ARM_FEED_FORWARD_KS = 0;
    public static final double ARM_FEED_FORWARD_KG = 0;
    public static final double ARM_FEED_FORWARD_KV = 0;

    public static final double GOTOANGLE_MOTOR_POWER = 0.4;

    public static final double TOLERANCE_DEGREES = 1;
}
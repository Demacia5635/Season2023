package frc.robot.subsystems.parallelogram;

/**
 * Parallelogram constants.
 */
public final class ParallelConstants {
    public static final int PORT_NUMBER_PARALLEL_MOTOR = -2;
    public static final double PARALLEL_LENGTH = -1; // In meters
    public static final double ROBOT_HEIGHT = 0;

    public static final double PULSE_PER_ROTATION_MOTOR = 2048;
    public static final double GEAR_RATIO = 0;
    public static final double PULSE_PER_ROTATION_PULI = GEAR_RATIO * PULSE_PER_ROTATION_MOTOR;
    public static final double PULSE_PER_ANGLE = PULSE_PER_ROTATION_PULI / 360;

    public static final double PULI_RADIUS = 1;
    public static final double PULI_PERIMETER = 2 * PULI_RADIUS * Math.PI;
    public static final double PULSE_PER_METER = PULSE_PER_ROTATION_PULI / PULI_PERIMETER;

    public static final double KS_VELOCITY = 0;
    public static final double KV_VELOCITY = 0;
}
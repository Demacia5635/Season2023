package frc.robot.subsystems.parallelogram;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Parallelogram constants.
 */
public final class ParallelConstants {
    public static final int PORT_NUMBER_PARALLEL_MOTOR = 30;
    public static final boolean MOTOR_INVERT_TYPE = true;
    public static final double PARALLEL_LENGTH = 0.7; // In meters
    public static final double ROBOT_HEIGHT = 0.58;

    public static final int GYRO_PORT_NUMBER = -4;
    public static final double GYRO_OFFSET = 0;

    public static final double PULSE_PER_ROTATION = 2048;
    public static final double GEAR_RATIO = 180;
    public static final double PULSE_PER_ANGLE = PULSE_PER_ROTATION*GEAR_RATIO/360;

    public static final double PULI_RADIUS = 1;
    public static final double PULI_PERIMETER = 2 * PULI_RADIUS * Math.PI;

    public static final double KS_VELOCITY = 0.0819816846419529; 
    public static final double KV_VELOCITY = 0.00505432515029668;

    public static final double DIGITAL_INPUT_ANGLE = 125.76;
    public static final int PORT_DIGITAL_INPUT = 0;

    public static final double KP_POSITION = 0.043;
    public static final double KI_POSITION = 0;
    public static final double KD_POSITION = 0;

    public static final double KP_VELOCITY = 0.1; 
    public static final double KI_VELOCITY = 0;
    public static final double KD_VELOCITY = 0.0;

    public static final double CALIBRATION_POWER = 0.2;
    public static final double END_POWER = -0.2;
    public static final double BACKWARDS_CALIBRATION_POWER = -0.2;

    public static final double ARM_FEED_FORWARD_KS = 0;
    public static final double ARM_FEED_FORWARD_KG = 0;
    public static final double ARM_FEED_FORWARD_KV = 0;

    public static final double GOTOANGLE_MOTOR_POWER = 0.4;

    public static final double TOLERANCE_DEGREES = 1;

    public static final double MAX_VELOCITY = 160; //degrees per sec
    public static final double MAX_ACCELERATION = 90;
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
    new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    
    public static final double PRECENTAGE_GOTOANGLE = 0.1;

}
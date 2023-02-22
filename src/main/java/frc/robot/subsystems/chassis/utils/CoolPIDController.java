package frc.robot.subsystems.chassis.utils;

import java.util.function.DoubleConsumer;

import edu.wpi.first.math.controller.PIDController;

/**
 * A PIDController that calls a function when it is used.
 */
public class CoolPIDController extends PIDController {
    private final DoubleConsumer onUse;

    /**
     * Constructs a new PIDController with the given constants for Kp, Ki, and Kd.
     * 
     * @param kp    The proportional coefficient.
     * @param ki    The integral coefficient.
     * @param kd    The derivative coefficient.
     * @param onUse The function to call when the controller is used, passes the
     *              set-point as a parameter.
     */
    public CoolPIDController(double kp, double ki, double kd, DoubleConsumer onUse) {
        super(kp, ki, kd);
        this.onUse = onUse;
    }

    @Override
    public double calculate(double measurement) {
        onUse.accept(getSetpoint());
        return super.calculate(measurement);
    }
}

package frc.robot.subsystems.parallelogram;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Paralellogram subsystem.
 */
public class Parallelogram extends SubsystemBase {

    private TalonFX motor;
    private SimpleMotorFeedforward feedForwardVelocity;

    /**
     * constructs a new parallelogram
     */
    public Parallelogram() {
        motor = new TalonFX(ParallelConstants.PORT_NUMBER_PARALLEL_MOTOR);
        feedForwardVelocity = new SimpleMotorFeedforward(ParallelConstants.KS_VELOCITY, ParallelConstants.KV_VELOCITY);
    }

    /**
     * Sets power to the paralellogram's motor.
     * 
     * @param power Desired power.
     */
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, velocity / 10 * ParallelConstants.PULSE_PER_METER,
                DemandType.ArbitraryFeedForward, feedForwardVelocity.calculate(velocity));
    }

    public double getVelocity() {
        return motor.getSelectedSensorVelocity() / ParallelConstants.PULI_PERIMETER;
    }

    public double getPosition() {
        return motor.getSelectedSensorPosition() / ParallelConstants.PULSE_PER_METER;
    }

    public void setPosition(double value) {

    }

}

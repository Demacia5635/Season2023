package frc.robot.subsystems.parallelogram;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    /**
     * Sets velocity to parallelogram's motor
     * 
     * @param velocity the velocity we want the motor have.
     */
    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, velocity / 10 * ParallelConstants.PULSE_PER_METER,
                DemandType.ArbitraryFeedForward, feedForwardVelocity.calculate(velocity));
    }

    // TODO: Calculations are wrong
    public double getVelocity() {
        return motor.getSelectedSensorVelocity() * ParallelConstants.PULI_PERIMETER / 10; //???
    }

    /**
     * Using the PPM calculations, we get the position the robot/parallelogram is
     * currently at.
     * 
     * @return current position in meters
     */
    public double getPosition() {
        return motor.getSelectedSensorPosition() / ParallelConstants.PULSE_PER_METER;
    }

    /** TODO: please check, very unsure :/
     * Should set the position of the parallelogram(?).
     * @param value is where we want the parallelogram to be (in meters...?).
     */
    public void setPosition(double value) {
        motor.setSelectedSensorPosition(value*ParallelConstants.PULSE_PER_METER);
    }

    /**
     * Returns the current angle (?)
     * 
     * @param height is the height measured from the robot to the wanted node height
     * @return current angle
     */
    public double getAngle(double height) {
        return Utils.calculateAngle(height);
    }

    /**
     * Sets motor to brake (?)
     */
    public void setBrakes() {
        motor.setNeutralMode(NeutralMode.Brake);
    }

}

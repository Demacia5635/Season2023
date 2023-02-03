package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ParallelConstants;

/**
 * Paralellogram subsystem.
 */
public class Parallelogram extends SubsystemBase {
    
    private TalonFX motor;
    private SimpleMotorFeedforward feedForwardVelocity;

    /**
     * constructs a new parallelogram
     */
    public Parallelogram(){
        motor = new TalonFX(ParallelConstants.PORT_NUMBER_PARALLEL_MOTOR);
        feedForwardVelocity = new SimpleMotorFeedforward(Constants.ParallelConstants.KS_VELOCITY, Constants.ParallelConstants.KV_VELOCITY);
    } 

    /**
     * Sets power to the paralellogram's motor.
     * 
     * @param power Desired power.
     */
    public void setPower(double power){
        motor.set(ControlMode.PercentOutput, power);
    }

    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, velocity/10*Constants.ParallelConstants.PULSE_PER_METER, 
        DemandType.ArbitraryFeedForward, feedForwardVelocity.calculate(velocity));
    }

    public double getVelocity() {
        return motor.getSelectedSensorVelocity()/Constants.ParallelConstants.PULI_PERIMETER;
    }

    public double getPosition() {
        return motor.getSelectedSensorPosition() / Constants.ParallelConstants.PULSE_PER_METER;
    } 

    public void setPosition(double value){
         
    }

}

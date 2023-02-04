package frc.robot.subsystems.parallelogram;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.parallelogramCommands.CalibrateParallelogram;

/**
 * Paralellogram subsystem.
 */
public class Parallelogram extends SubsystemBase {

    private TalonFX motor;
    private SimpleMotorFeedforward feedForwardVelocity;
    private DigitalInput magneticDigitalInput;
    private boolean isBrake;

    /**
     * constructs a new parallelogram
     */
    public Parallelogram() {
        motor = new TalonFX(ParallelConstants.PORT_NUMBER_PARALLEL_MOTOR);
        magneticDigitalInput  = new DigitalInput(ParallelConstants.PORT_DIGITAL_INPUT);
        feedForwardVelocity = new SimpleMotorFeedforward(ParallelConstants.KS_VELOCITY, ParallelConstants.KV_VELOCITY);
        
        motor.config_kP(0, ParallelConstants.KP_POSITION);
        motor.config_kI(0, ParallelConstants.KI_POSITION);
        motor.config_kD(0, ParallelConstants.KD_POSITION);

        isBrake = false;
        
        SmartDashboard.putData("Calibrate Parallelogram", new CalibrateParallelogram(this));
        SmartDashboard.putNumber("wanted angle", 0);
        SmartDashboard.putData("set angle", new InstantCommand(() -> { setAngle(SmartDashboard.getNumber("wanted angle", 0));}));
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


    public double getVelocity() {
        return motor.getSelectedSensorVelocity() * 10 / ParallelConstants.PULSE_PER_METER;
    }


    /**
     * Sets the position of the arm.
     * 
     * @param position is where we want the parallelogram to be.
     */
    public void setAngle(double position) {
        motor.set(ControlMode.Position, position * ParallelConstants.PULSE_PER_ANGLE);
    }

    /**
     * Gets current angle of the arm.
     * 
     * @param height is the height measured from the robot to the wanted node height
     * @return current angle
     */
    public double getAngle() {
        return motor.getSelectedSensorPosition() / ParallelConstants.PULSE_PER_ANGLE;
    }

    /**
     * Sets motor neutral mode to brake.
     */
    public void setBrake() {
        motor.setNeutralMode(NeutralMode.Brake);
        isBrake = true;
    }

    /**
     *  Sets motor neutral mode to coast.
     */
    public void setCoast() {
        motor.setNeutralMode(NeutralMode.Coast);
        isBrake = false;
    }

    /**
     * TODO:
     * @return
     */
    public boolean getDigitalInput() {
        return magneticDigitalInput.get();
    }

    public void resetPosition() {
        motor.setSelectedSensorPosition(ParallelConstants.DIGITAL_INPUT_ANGLE * ParallelConstants.PULSE_PER_ANGLE);
    }

    public boolean isBrake(){
        return isBrake;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("is Digital Input", getDigitalInput());
        SmartDashboard.putBoolean("is Brake", isBrake);
        SmartDashboard.putData(resetPosition());
        SmartDashboard.putNumber("arm angle", getAngle());
        SmartDashboard.putNumber("pulli velocity", getVelocity());
    }

}

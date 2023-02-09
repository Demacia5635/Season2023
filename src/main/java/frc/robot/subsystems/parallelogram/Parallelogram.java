package frc.robot.subsystems.parallelogram;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.parallelogram.CalibrateParallelogram;

/**
 * Paralellogram subsystem.
 */
public class Parallelogram extends SubsystemBase {

    private TalonFX motor;
    private SimpleMotorFeedforward feedForwardVelocity;
    private ArmFeedforward armFeedForward;
    private DigitalInput magneticDigitalInput;
    private boolean isBrake;

    /**
     * constructs a new parallelogram
     */
    public Parallelogram() {
        motor = new TalonFX(ParallelConstants.PORT_NUMBER_PARALLEL_MOTOR);
        magneticDigitalInput = new DigitalInput(ParallelConstants.PORT_DIGITAL_INPUT);
        feedForwardVelocity = new SimpleMotorFeedforward(ParallelConstants.KS_VELOCITY, ParallelConstants.KV_VELOCITY);
        armFeedForward = new ArmFeedforward(ParallelConstants.ARM_FEED_FORWARD_KS,
                ParallelConstants.ARM_FEED_FORWARD_KG, ParallelConstants.ARM_FEED_FORWARD_KV);

        motor.config_kP(0, ParallelConstants.KP_POSITION);
        motor.config_kI(0, ParallelConstants.KI_POSITION);
        motor.config_kD(0, ParallelConstants.KD_POSITION);

        motor.config_kP(1, ParallelConstants.KP_VELOCITY);
        motor.config_kI(1, ParallelConstants.KI_VELOCITY);
        motor.config_kD(1, ParallelConstants.KD_VELOCITY);

        isBrake = false;

        SmartDashboard.putData("Calibrate Parallelogram", new CalibrateParallelogram(this));
        SmartDashboard.putNumber("wanted angle", 0);
        SmartDashboard.putData("set angle", new InstantCommand(() -> {
            setAngle(SmartDashboard.getNumber("wanted angle", 0));
        }));
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
        motor.selectProfileSlot(1, 0);
        motor.set(ControlMode.Velocity, velocity / 10 * ParallelConstants.PULSE_PER_METER,
                DemandType.ArbitraryFeedForward, feedForwardVelocity.calculate(velocity));
    }

    /**
     * Gets the velocity.
     * 
     * @return the velocity.
     */
    public double getVelocity() {
        motor.selectProfileSlot(0, 0);
        return motor.getSelectedSensorVelocity() * 10 / ParallelConstants.PULSE_PER_METER;
    }

    /**
     * Sets the position of the arm.
     * 
     * @param angle is where we want the parallelogram to be.
     */
    public void setAngle(double angle) {
        motor.set(ControlMode.Position, angle * ParallelConstants.PULSE_PER_ANGLE, DemandType.ArbitraryFeedForward,
                armFeedForward.calculate(Utils.toRads(getAngle()), getVelocity()));
    }

    //TODO: Upon seeing the ArmFeedForward, I think we should try no feed forward first! ~ Noya

    /**
     * Gets current angle of the arm.
     * 
     * @param height is the height measured from the robot to the wanted node height
     * @return current angle
     */
    public double getAngle() {
        if (getDigitalInput()) {
            resetPosition();
        }
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
     * Sets motor neutral mode to coast.
     */
    public void setCoast() {
        motor.setNeutralMode(NeutralMode.Coast);
        isBrake = false;
    }

    /**
     * Gets the digital input from the magnet on the parallelogram.
     * 
     * @return whether the arm reached the magnet or not (boolean).
     */
    public boolean getDigitalInput() {
        return magneticDigitalInput.get();
    }

    /**
     * Sets the current position/angle of the arm as the digital input angle (max
     * angle?).
     */
    public void resetPosition() {
        motor.setSelectedSensorPosition(ParallelConstants.DIGITAL_INPUT_ANGLE * ParallelConstants.PULSE_PER_ANGLE);
    }

    /**
     * checks if the motor's neutral mode is set on brake or coast.
     * 
     * @return the neutral mode of the motor (true = brake).
     */
    public boolean isBrake() {
        return isBrake;
    }

    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("is Digital Input", getDigitalInput());
        SmartDashboard.putBoolean("is Brake", isBrake);

        SmartDashboard.putNumber("arm angle", getAngle());
        SmartDashboard.putNumber("pulli velocity", getVelocity());
    }

}

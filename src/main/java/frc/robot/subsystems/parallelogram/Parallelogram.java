package frc.robot.subsystems.parallelogram;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.parallelogram.CalibrateParallelogram;
import frc.robot.commands.parallelogram.GoToAngle;
import frc.robot.commands.parallelogram.GoToHeight;
import frc.robot.commands.parallelogram.ResetCalibrate;
import frc.robot.commands.parallelogram.TrapezoidGoToAngle;

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

        SmartDashboard.putNumber("desired angle", 0);

        motor = new TalonFX(ParallelConstants.PORT_NUMBER_PARALLEL_MOTOR);
        magneticDigitalInput = new DigitalInput(ParallelConstants.PORT_DIGITAL_INPUT);
        feedForwardVelocity = new SimpleMotorFeedforward(ParallelConstants.KS_VELOCITY, ParallelConstants.KV_VELOCITY);
        armFeedForward = new ArmFeedforward(ParallelConstants.ARM_FEED_FORWARD_KS,
                ParallelConstants.ARM_FEED_FORWARD_KG, ParallelConstants.ARM_FEED_FORWARD_KV);

        motor.setInverted(ParallelConstants.MOTOR_INVERT_TYPE);

        motor.config_kP(0, ParallelConstants.KP_VELOCITY);
        motor.config_kI(0, ParallelConstants.KI_VELOCITY);
        motor.config_kD(0, ParallelConstants.KD_VELOCITY);

        isBrake = false;

        SmartDashboard.putData("Parallelogram/Calibrate Parallelogram",
                new CalibrateParallelogram(this));
        SmartDashboard.putData("Parallelogram/Go to angle",
                new GoToAngle(this, 90));
        SmartDashboard.putData("Parallelogram/trapezoid",
                new TrapezoidGoToAngle(this, 90));

        SmartDashboard.putData("Parallelogram/Go to height",
                new GoToHeight(this, ParallelConstants.PARALLEL_LENGTH, true));

        SmartDashboard.putData(this);
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
     * @param velocity the velocity we want the motor have in degrees per seconds.
     */
    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, velocity / 10 * ParallelConstants.PULSE_PER_ANGLE,
                DemandType.ArbitraryFeedForward, feedForwardVelocity.calculate(velocity));
                //TODO: not sure if arm ff or simple motor ~ Noya
    }

    /**
     * Gets the velocity.
     * 
     * @return the velocity in degrees per second.
     */
    public double getVelocity() {
        return motor.getSelectedSensorVelocity() * 10 / ParallelConstants.PULSE_PER_ANGLE;
    }

    /**
     * Sets the position of the arm.
     * 
     * @param angle is where we want the parallelogram to be.
     */
    public void setAngle(double angle) {
        motor.set(ControlMode.Position, angle * ParallelConstants.PULSE_PER_ANGLE, DemandType.ArbitraryFeedForward,
                armFeedForward.calculate(Utils.toRads(angle), 0));
    }

    /**
     * Gets the angle of the arm.
     * 
     * @return the arm's angle (after calculating with offset).
     */
    public double getAngle() {
        if (getDigitalInput()) {
            resetPosition();
        }
        return motor.getSelectedSensorPosition() / ParallelConstants.PULSE_PER_ANGLE;
    }

    // public double getOffset(){
    // return gyro.getPitch() - 90 ;
    // }

    /*
     * Resets the offset for the gyro.
     */


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
        return !magneticDigitalInput.get();
    }

    /**
     * Sets the current position/angle of the arm as the digital input angle (max
     * angle?).
     */
    public void resetPosition() {
        motor.setSelectedSensorPosition(ParallelConstants.DIGITAL_INPUT_ANGLE * ParallelConstants.PULSE_PER_ANGLE);
    }

    public void resetPosition(double angle) {
        motor.setSelectedSensorPosition(angle * ParallelConstants.PULSE_PER_ANGLE);
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
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("encoder", motor::getSelectedSensorPosition, null);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("parallelogram/is Digital Input", getDigitalInput());
        SmartDashboard.putBoolean("parallelogram/is Brake", isBrake);

        SmartDashboard.putNumber("parallelogram/Arm angle", getAngle());
        SmartDashboard.putNumber("parallelogram/arm velocity", getVelocity());

        SmartDashboard.putNumber("parallelogram/sensot position", motor.getSelectedSensorPosition());

    }

}

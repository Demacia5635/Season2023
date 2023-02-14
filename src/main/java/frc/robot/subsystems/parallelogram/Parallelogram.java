package frc.robot.subsystems.parallelogram;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.parallelogram.CalibrateParallelogram;
import frc.robot.commands.parallelogram.GoToAngle;
import frc.robot.commands.parallelogram.GoToHeight;

/**
 * Paralellogram subsystem.
 */
public class Parallelogram extends SubsystemBase {

    private TalonFX motor;
    private ArmFeedforward armFeedForward;
    private DigitalInput magneticDigitalInput;
    private boolean isBrake;

    /**
     * constructs a new parallelogram
     */
    public Parallelogram() {
        SmartDashboard.putNumber("wangle", 0);

        motor = new TalonFX(ParallelConstants.PORT_NUMBER_PARALLEL_MOTOR);
        magneticDigitalInput = new DigitalInput(ParallelConstants.PORT_DIGITAL_INPUT);
        armFeedForward = new ArmFeedforward(ParallelConstants.ARM_FEED_FORWARD_KS,
                ParallelConstants.ARM_FEED_FORWARD_KG, ParallelConstants.ARM_FEED_FORWARD_KV);

        motor.setInverted(ParallelConstants.MOTOR_INVERT_TYPE);

        motor.config_kP(0, ParallelConstants.KP_POSITION);
        motor.config_kI(0, ParallelConstants.KI_POSITION);
        motor.config_kD(0, ParallelConstants.KD_POSITION);

        motor.configMotionCruiseVelocity(ParallelConstants.CRUISE_VELOCITY_SU);
        motor.configMotionAcceleration(ParallelConstants.MAX_ACCELERATION_SU);

        isBrake = false;

        resetPosition(ParallelConstants.DIGITAL_INPUT_ANGLE);

        SmartDashboard.putNumber("wanted angle", 0);
        SmartDashboard.putNumber("wanted power", 0);

        SmartDashboard.putData("Calibrate Parallelogram", new CalibrateParallelogram(this));

        SmartDashboard.putData("Go to 55", 
        new GoToAngle(this, 55));

        SmartDashboard.putData("Go to 90", 
        new GoToAngle(this, 90));

        SmartDashboard.putData("all", new GoToAngle(this, 0));

        // SmartDashboard.putData("go to angle",
        // new InstantCommand(()-> setAngle(SmartDashboard.getNumber("wanted angle", 0))));

        SmartDashboard.putData("find ff",
        new InstantCommand(()-> setPower(-0.7))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(()-> {setPower(0); SmartDashboard.putNumber("angular velocity", getVelocity());
        SmartDashboard.putNumber("angle ff", getAngle());})));

        SmartDashboard.putData("Go to end", 
        new GoToAngle(this, ParallelConstants.DIGITAL_INPUT_ANGLE));

        SmartDashboard.putData("go to height", 
        new GoToHeight(this, 0.95, true));

        //ParallelConstants.PARALLEL_LENGTH+ParallelConstants.ROBOT_HEIGHT
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
     * @param velocity the velocity we want the motor have in degrees per second.
     */
    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, velocity /10 * ParallelConstants.PULSE_PER_ANGLE);
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
     * Calculates arm feed forward
     * @param angle the desired angle in degrees
     * @param velocity the desired radial velocity
     * @return feed forward
     */
    public double calculateFF(double angle, double velocity) {
        velocity=velocity*10/ParallelConstants.PULSE_PER_ANGLE;
        return ParallelConstants.KS_MM*Math.signum(velocity) +
        ParallelConstants.KG_MM*Math.cos(Utils.toRads(angle))*Math.signum(-velocity) +
        ParallelConstants.KV_MM*velocity;
    }

    /**
     * Sets the position of the arm.
     * 
     * @param angle the parallelogram angle.
     */
    public void setAngle(double angle) {
        motor.set(ControlMode.MotionMagic, angle,
        DemandType.ArbitraryFeedForward, calculateFF(angle, motor.getActiveTrajectoryVelocity()));
    }

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
     * Sets motor position
     * @param pulse position in sensor units
     */
    public void setPosition(double pulse) {
        motor.set(ControlMode.Position, pulse);
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
        
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("parallelogram/is Digital Input", getDigitalInput());
        SmartDashboard.putBoolean("parallelogram/is Brake", isBrake);

        SmartDashboard.putNumber("parallelogram/Arm angle", getAngle());
        SmartDashboard.putNumber("parallelogram/pulli velocity", getVelocity());

        // SmartDashboard.putNumber("arm velocity su", motor.getSelectedSensorVelocity());

        // SmartDashboard.putNumber("closed-loop error", motor.getClosedLoopError());

        SmartDashboard.putNumber("sensor pose", motor.getSelectedSensorPosition());
    }

}

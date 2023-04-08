package frc.robot.subsystems.parallelogram;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.parallelogram.CalibrateParallelogram;
import frc.robot.commands.parallelogram.EndGoToAngle;
import frc.robot.commands.parallelogram.GoToAngle;
import frc.robot.commands.parallelogram.ResetCalibrate;
import frc.robot.subsystems.chassis.Chassis;

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

        motor.setInverted(ParallelConstants.MOTOR_INVERT_TYPE);


        motor.config_kP(0, ParallelConstants.KP_POSITION);
        motor.config_kI(0, ParallelConstants.KI_POSITION);
        motor.config_kD(0, ParallelConstants.KD_POSITION);
        
        isBrake = false;
        
        // SmartDashboard.putData("Parallelogram/Calibrate Parallelogram",
        //         getCalibrationCommand());
        SmartDashboard.putData("Parallelogram/Go to angle",
                getGoToAngleCommand(30));
        // SmartDashboard.putData("Parallelogram/go back",
        //         getGoBackCommand());
        SmartDashboard.putData("Parallelogram/check",
                getGoToAngleCommand(120));

        new Thread(() -> {
            Timer timer = new Timer();
            timer.start();
            while (timer.get() < 2);
            resetPosition();
        }).start();
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
     * @param velocity the velocity we want the motor have in degrees per second.
     */
    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, velocity / 10 * ParallelConstants.PULSE_PER_ANGLE,
                DemandType.ArbitraryFeedForward, feedForwardVelocity.calculate(velocity));
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
                armFeedForward.calculate(ParallelogramUtils.toRads(angle), 0));
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
     * Creates and returns go back command.
     * @return go back command.
     */
    public CommandBase getGoBackCommand() {
        return new SequentialCommandGroup(new GoToAngle(this, 115),
        new CalibrateParallelogram(this), new ResetCalibrate(this));
    }

    /**
     * Creates and returns calibration command.
     * @return calibration command.
     */
    public CommandBase getCalibrationCommand(Chassis chassis) {
        return new CalibrateParallelogram(this).andThen(new ResetCalibrate(this));
    }

    /**
     * Creates and returns go to angle command.
     * @param angle the desired angle of the parallelogram.
     * @return go to angle command.
     */
    public CommandBase getGoToAngleCommand(double angle) {
        return new GoToAngle(this, angle +
        ParallelConstants.PRECENTAGE_GOTOANGLE*(ParallelConstants.DIGITAL_INPUT_ANGLE-angle))
        .andThen(new EndGoToAngle(this, angle));
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

    }

}

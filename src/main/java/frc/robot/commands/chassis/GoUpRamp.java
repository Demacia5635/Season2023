package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.UtilsGeneral;

/**
 * This command is used to go up the ramp.
 */
public class GoUpRamp extends CommandBase {

    private final Chassis chassis;
    private final double startVelocity;
    private double velocity;
    private double angleSign;
    private State lastState;
    private boolean onRamp;
    private int count;
    private double VEL_MIN_COUNTER;


    private static final double DEADBAND = 1;
    private static final double ROTATION_MINIMUM = 5; // the minimum angle to be on the ramp while not straight
    private static final int COUNT_MINIMUM = 50; // the minimum count to be on the ramp to stop the robot
    private static final double VELOCITY_FACTOR = 4; // the factor to multiply the velocity by to go up the ramp
    private static final double MIN_VELOCITY = 0;
    private static final double FIRST_ROTATION_MINIMUM = 10;

    /**
     * Creates a new GoUpRamp command.
     * 
     * @param chassis  The chassis subsystem
     * @param velocity The velocity to go up the ramp
     */
    public GoUpRamp(Chassis chassis, double velocity) {
        this.chassis = chassis;
        this.startVelocity = UtilsGeneral.isRedAlliance() ? velocity : -velocity;
        VEL_MIN_COUNTER = 0;

        addRequirements(chassis);
    }

    /**
     * Deadbands the angle to prevent the robot from going up and down the ramp.
     * 
     * @param angle The angle to deadband
     * @return The deadbanded angle
     */
    private static double deadbandAngle(double angle) {
        if (Math.abs(angle) < DEADBAND) {
            return 0;
        }
        return angle;
    }

    /**
     * The state of the robot, either positive angle or negative angle.
     */
    private enum State {
        POSITIVE,
        NEGATIVE,
        UNKNOWN;

        /**
         * Checks if the sign is different from the state.
         * 
         * @param angle The angle to check
         * @return True if the sign is different, false otherwise
         */
        public boolean differentSign(double angle) {
            angle = Math.signum(deadbandAngle(angle));
            return (angle == 1 && this == NEGATIVE) ||
                    (angle == -1 && this == POSITIVE);
        }
    }

    @Override
    public void initialize() {
        onRamp = false;
        velocity = startVelocity;
        lastState = State.UNKNOWN;
    }

    @Override
    public void execute() {
        double angle = chassis.getUpRotation();
        chassis.setAngleAndVelocity((onRamp && angleSign == 0) ? 0 : velocity, 0, Math.toRadians(UtilsGeneral.isRedAlliance() ? 45 : 235));

        if (!onRamp && Math.abs(angle) > FIRST_ROTATION_MINIMUM) {
            onRamp = true;
            velocity /= 3.5;
        }
        else if (onRamp && lastState.differentSign(angle))
            velocity /= -VELOCITY_FACTOR;
            if (Math.abs(velocity) < MIN_VELOCITY)
                velocity = MIN_VELOCITY * Math.signum(velocity);

        angleSign = Math.signum(deadbandAngle(angle));
        if (angleSign == 1)
            lastState = State.POSITIVE;
        else if (angleSign == -1)
            lastState = State.NEGATIVE;

        if (angleSign == 0 && onRamp) {
            count++;
        } else {
            count = 0;
        }
        if(velocity == MIN_VELOCITY){
            VEL_MIN_COUNTER++;
        }
        if(VEL_MIN_COUNTER > 90){
            velocity = 0;
            new StartEndCommand(chassis::setRampPosition, ()-> System.out.println("climb ended"), chassis).schedule(); 
        }

    }

    @Override
    public boolean isFinished() {
        return count >= COUNT_MINIMUM;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
}

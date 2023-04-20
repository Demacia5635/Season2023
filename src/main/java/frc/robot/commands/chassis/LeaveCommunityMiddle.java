package frc.robot.commands.chassis;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.UtilsGeneral;

public class LeaveCommunityMiddle extends CommandBase {
    private final static double INITIAL_VELOCITY = 1.8;
    public static final int SAMPLE_NUM= 5;
    private final static double ON_ANGLE = 12;

    MedianFilter medianFilter;
    boolean wasDown;
    boolean isDone;
    double upRotation;
    Chassis chassis;
    double velocity;
    double direction;
    boolean isOnRamp;


    public LeaveCommunityMiddle(Chassis chassis) {

        medianFilter = new MedianFilter(SAMPLE_NUM);
        wasDown = false;
        isDone = false;
        this.chassis = chassis;
        isOnRamp = false;

    }

    @Override
    public void initialize() {
        velocity = UtilsGeneral.isRedAlliance() ? INITIAL_VELOCITY : -INITIAL_VELOCITY;
    }

    @Override
    public void execute() {

        chassis.setAngleAndVelocity(velocity, 0, UtilsGeneral.isRedAlliance() ? 0 : Math.toRadians(180));

        upRotation = medianFilter.calculate(chassis.getUpRotation());
        if (!isOnRamp && Math.abs(upRotation) > ON_ANGLE) {
            isOnRamp = true;
            direction = Math.signum(upRotation);
        }

        if (Math.signum(upRotation)!=direction) {
            wasDown = true;
        }

        if ((deadBand(Math.signum(upRotation))==0) && (wasDown)) {
            isDone = true;
        }
        
        
    }


    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }

    public double deadBand(double value) {
        if (Math.abs(value)<1) value = 0;
        return value;
    }

    
    
}

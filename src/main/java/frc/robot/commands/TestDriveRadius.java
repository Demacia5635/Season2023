package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;

/**
 * A command that drives the robot forward, and then turns 90 degrees to the
 * left, printing the radius of the turn at the end.
 */
public class TestDriveRadius extends CommandBase {

    private final Chassis chassis;
    private static final double VELOCITY = 4;
    private boolean turned;
    private double x;

    /**
     * Creates a new TestDriveRadius command.
     * 
     * @param chassis the chassis subsystem
     */
    public TestDriveRadius(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        turned = false;
        chassis.setAngleAndVelocity(VELOCITY, 0, 0);
        x = 0;
    }

    @Override
    public void execute() {
        if (Math.abs(chassis.getVelocity().getX() - VELOCITY) < 0.1 && !turned) {
            chassis.setAngleAndVelocity(0, VELOCITY, 0);
            turned = true;
            x = chassis.getPose().getX();
        }
    }

    @Override
    public boolean isFinished() {
        return turned && Math.abs(chassis.getVelocity().getY() - VELOCITY) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
        if (!interrupted)
            SmartDashboard.putNumber("Drive Radius", chassis.getPose().getX() - x);
    }
}

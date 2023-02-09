package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;

public class TestDecceleration extends CommandBase {
    
    private final Chassis chassis;
    private static final double VELOCITY = 3.5;
    private int count;

    /**
     * Creates a new TestDecceleration command.
     * 
     * @param chassis the chassis subsystem
     */
    public TestDecceleration(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        count = -1;
        chassis.setAngleAndVelocity(VELOCITY, 0, 0);
    }

    @Override
    public void execute() {
        if (Math.abs(chassis.getVelocity().getX() - VELOCITY) < 0.1 && count < 0) {
            chassis.stop();
            count = 0;
        } else if (count >= 0)
            count++;
    }

    @Override
    public boolean isFinished() {
        return chassis.getVelocity().getNorm() < 0.1 && count >= 0;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
        if (!interrupted)
            SmartDashboard.putNumber("Decceleration", (VELOCITY / count) * 50);
    }
}

package frc.robot.commands.parallelogram;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class GoToAngle extends CommandBase {
    private Parallelogram parallelogram;
    private TrapezoidProfile trapezoidProfile;
    private TrapezoidProfile.State setPoint;
    private double desiredAngle;
    private Timer timer;
    
    public GoToAngle(Parallelogram parallelogram, double desiredAngle) {
        this.parallelogram = parallelogram;
        this.desiredAngle = desiredAngle;

        addRequirements(parallelogram);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.start();
        parallelogram.setBrake();
        trapezoidProfile = new TrapezoidProfile(ParallelConstants.CONSTRAINTS,
        new TrapezoidProfile.State(desiredAngle, 0),
        new TrapezoidProfile.State(parallelogram.getAngle(), 0));
        
    }

    @Override
    public void execute() {
        setPoint = trapezoidProfile.calculate(timer.get());
        parallelogram.setVelocity(setPoint.velocity);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(parallelogram.getAngle() - desiredAngle)
        < ParallelConstants.TOLERANCE_DEGREES || trapezoidProfile.isFinished(timer.get());
    }

    public void end(boolean interrupted) {
        parallelogram.setPower(0);
    }
    
}
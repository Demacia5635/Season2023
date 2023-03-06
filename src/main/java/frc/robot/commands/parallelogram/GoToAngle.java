package frc.robot.commands.parallelogram;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class GoToAngle extends CommandBase {
    private Parallelogram parallelogram;
    private TrapezoidProfile trapezoidProfile;
    private TrapezoidProfile.State setPoint;
    private double desiredAngle;
    private double seconds;
    private Timer timer;
    
    public GoToAngle(Parallelogram parallelogram, double desiredAngle) {
        this.parallelogram = parallelogram;
        this.desiredAngle = desiredAngle;
    }

    @Override
    public void initialize() {
        seconds = 0;
        timer = new Timer();
        parallelogram.setBrake();
        trapezoidProfile = new TrapezoidProfile(ParallelConstants.CONSTRAINTS,
        new TrapezoidProfile.State(desiredAngle, 0),
        new TrapezoidProfile.State(parallelogram.getAngle(), 0));
        
    }

    @Override
    public void execute() {
        seconds=seconds+0.02;
        setPoint = trapezoidProfile.calculate(seconds);
        parallelogram.setVelocity(setPoint.velocity);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(parallelogram.getAngle() - desiredAngle)
        < ParallelConstants.TOLERANCE_DEGREES || trapezoidProfile.isFinished(seconds);
    }

    public void end(boolean interrupted) {
        parallelogram.setPower(0);
    }
    
}
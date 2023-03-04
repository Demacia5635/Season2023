package frc.robot.commands.parallelogram;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class TrapezoidGoToAngle extends CommandBase {
    private Parallelogram parallelogram;
    private TrapezoidProfile trapezoidProfile;
    private TrapezoidProfile.State setPoint;
    private double desiredAngle;
    private double seconds;
    
    public TrapezoidGoToAngle(Parallelogram parallelogram, double desiredAngle) {
        this.parallelogram = parallelogram;
        this.desiredAngle = desiredAngle;
    }

    @Override
    public void initialize() {
        seconds = 0;
        parallelogram.setBrake();
        trapezoidProfile = new TrapezoidProfile(ParallelConstants.CONSTRAINTS,
        new TrapezoidProfile.State(desiredAngle, 0),
        new TrapezoidProfile.State(parallelogram.getAngle(), 0));
        
    }

    @Override
    public void execute() {
        seconds=seconds+0.02;
        SmartDashboard.putNumber("seconds of timer", Timer.getFPGATimestamp());
        SmartDashboard.putNumber("seconds", seconds);
        setPoint = trapezoidProfile.calculate(seconds);
        SmartDashboard.putNumber("setPoint", setPoint.velocity);
        SmartDashboard.putNumber("setPointPstn", setPoint.position);
        parallelogram.setVelocity(setPoint.velocity);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(parallelogram.getAngle() - desiredAngle) < ParallelConstants.TOLERANCE_DEGREES;
    }

    public void end(boolean interrupted) {
        SmartDashboard.putNumber("cehck", 50);
        parallelogram.setPower(0);
    }
    
}

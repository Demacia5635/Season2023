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
    
    public TrapezoidGoToAngle(Parallelogram parallelogram, double desiredAngle) {
        this.parallelogram = parallelogram;
        this.desiredAngle = desiredAngle;
        trapezoidProfile = new TrapezoidProfile(ParallelConstants.CONSTRAINTS,
        new TrapezoidProfile.State(ParallelConstants.DIGITAL_INPUT_ANGLE, 0),
        new TrapezoidProfile.State(desiredAngle, 0));
    }

    @Override
    public void initialize() {
        parallelogram.setBrake();
        desiredAngle = SmartDashboard.getNumber("desired angle", 90);
        trapezoidProfile = new TrapezoidProfile(ParallelConstants.CONSTRAINTS,
        new TrapezoidProfile.State(ParallelConstants.DIGITAL_INPUT_ANGLE, 0),
        new TrapezoidProfile.State(desiredAngle, 0));
        //lines 27-31 should be deleted when testing is over.
    }

    @Override
    public void execute() {
        setPoint = trapezoidProfile.calculate(Timer.getFPGATimestamp());
        parallelogram.setVelocity(setPoint.velocity);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(parallelogram.getAngle() - desiredAngle) < ParallelConstants.TOLERANCE_DEGREES;
    }

    public void end(boolean interrupted) {
        parallelogram.setPower(0);
    }
    
}

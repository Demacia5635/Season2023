package frc.robot.commands.parallelogram;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class TrapezoidGoToAngle extends CommandBase {
    private Parallelogram parallelogram;
    private TrapezoidProfile trapezoidProfile;
    private TrapezoidProfile.State setPoint;
    
    public TrapezoidGoToAngle(Parallelogram parallelogram, double desiredAngle) {
        this.parallelogram = parallelogram;
        trapezoidProfile = new TrapezoidProfile(ParallelConstants.CONSTRAINTS,
        new TrapezoidProfile.State(ParallelConstants.DIGITAL_INPUT_ANGLE, 0),
        new TrapezoidProfile.State(desiredAngle, 0));
    }

    @Override
    public void initialize() {
        parallelogram.setBrake();
    }

    @Override
    public void execute() {
        setPoint = trapezoidProfile.calculate(Timer.getFPGATimestamp());
        parallelogram.setVelocity(0);
    }
    
}

package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class EndGoToAngle extends CommandBase{

    private Parallelogram parallelogram;
    private double desiredAngle;

    public EndGoToAngle(Parallelogram parallelogram, double desiredAngle) {
        this.parallelogram = parallelogram;
        this.desiredAngle = desiredAngle;

        addRequirements(parallelogram);
    }

    @Override
    public void initialize() {
        parallelogram.setBrake();
    }

    @Override
    public void execute(){
        parallelogram.setPower(ParallelConstants.END_POWER);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(parallelogram.getAngle() - desiredAngle)
        < ParallelConstants.TOLERANCE_DEGREES;
    }
    

    public void end(boolean interrupted) {
        parallelogram.setPower(0);
    }
}

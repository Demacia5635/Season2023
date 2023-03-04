package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class GoToAngle extends CommandBase {
    private Parallelogram parallelogram;
    private double desiredAngle;
    private boolean isFinished;

    /**
     * Command's constructor.
     * 
     * @param parallelogram
     */
    public GoToAngle(Parallelogram parallelogram, double desiredAngle) {
        this.parallelogram = parallelogram;
        this.desiredAngle = desiredAngle;

        addRequirements(parallelogram);
    }

    @Override
    public void initialize() {
        isFinished = false;
        parallelogram.setBrake();
    }

    @Override
    public void execute() {
        double diff = desiredAngle - parallelogram.getAngle();
        if (Math.abs(diff) < 10) {
            parallelogram.setPower(Math.signum(diff) * 0.3);
        } else {
            parallelogram.setPower(Math.signum(diff) * ParallelConstants.GOTOANGLE_MOTOR_POWER);
        }

    }

    @Override
    public boolean isFinished() {
        if (!isFinished)
            isFinished = parallelogram.getAngle() - desiredAngle < ParallelConstants.TOLERANCE_DEGREES;
        return isFinished;
    }

    public boolean getFinished() {
        return isFinished;
    }

    public void end(boolean interrupted) {
        parallelogram.setPower(0);
        System.out.println("parallelogram Go To Angle Ended");
    }

}

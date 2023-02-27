package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class GoToAngle extends CommandBase {
    private Parallelogram parallelogram;
    private double desiredAngle;
    private boolean isFinished;
    //TODO : REMOVE THIS TEST
    private double testAngle;

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
        testAngle = parallelogram.getAngle();
        isFinished = false;
        parallelogram.setBrake();
    }

    @Override
    public void execute() {
        double diff = desiredAngle - parallelogram.getAngle();
        if (Math.abs(diff) < 5) {
            parallelogram.setPower(Math.signum(diff) * 0.1);
        } else {
            parallelogram.setPower(Math.signum(diff) * ParallelConstants.GOTOANGLE_MOTOR_POWER);
        }

    }

    @Override
    public boolean isFinished() {
        System.out.println("Activation Value: " + testAngle +  " Current Angle: " + parallelogram.getAngle() + " Desired Angle: " + desiredAngle);
        if (!isFinished)
            isFinished = parallelogram.getAngle() - desiredAngle < ParallelConstants.TOLERANCE_DEGREES;
        return isFinished;
    }

    public boolean getFinished() {
        return isFinished;
    }

    public void end(boolean interrupted) {
        parallelogram.setPower(0);
    }

}

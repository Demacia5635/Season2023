package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;
import frc.robot.subsystems.parallelogram.ParallelogramUtils;

public class GoToHeight extends CommandBase{
    private Parallelogram parallelogram;
    private double desiredAngle;

    /**
     * Command's constructor.
     * @param parallelogram
     */
    public GoToHeight(Parallelogram parallelogram, double desiredHeight, boolean isFront) {
        this.parallelogram = parallelogram;
        this.desiredAngle = ParallelogramUtils.calculateAngle(desiredHeight, isFront);
    }

    @Override
    public void initialize() {
        parallelogram.setBrake();
    }


    @Override
    public void execute() {
        if(parallelogram.getAngle()-desiredAngle>0) {
            parallelogram.setPower(-ParallelConstants.GOTOANGLE_MOTOR_POWER);
        }
        else {
            parallelogram.setPower(ParallelConstants.GOTOANGLE_MOTOR_POWER);
        }
       
    }

    @Override
    public boolean isFinished() {
        return Math.abs(parallelogram.getAngle()-desiredAngle)<ParallelConstants.TOLERANCE_DEGREES;
    }

    public void end(boolean interrupted) {
        parallelogram.setPower(0);
    }
}

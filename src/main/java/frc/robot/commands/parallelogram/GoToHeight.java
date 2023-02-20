package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;
import frc.robot.subsystems.parallelogram.Utils;

public class GoToHeight extends CommandBase{
    private Parallelogram parallelogram;
    private double desiredAngle;
    private boolean isFront;

    /**
     * Command's constructor.
     * @param parallelogram
     */
    public GoToHeight(Parallelogram parallelogram, double desiredHeight, boolean isFront) {
        this.parallelogram = parallelogram;
        this.desiredAngle = Utils.calculateAngle(desiredHeight, isFront);
        this.isFront = isFront;
    }

    @Override
    public void initialize() {
        this.desiredAngle = Utils.calculateAngle(SmartDashboard.getNumber("Height", 0), isFront);
        SmartDashboard.putNumber("calculated angle", desiredAngle);
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
        SmartDashboard.putNumber("Parallelogram/Check current", parallelogram.getAngle());
        SmartDashboard.putNumber("Parallelogram/Check desired", desiredAngle);
        SmartDashboard.putNumber("Parallelogram/DIFFERENCE", Math.abs(parallelogram.getAngle()-desiredAngle));
        SmartDashboard.putBoolean("Parallelogram/isFinished", 
        Math.abs(parallelogram.getAngle()-desiredAngle)<ParallelConstants.TOLERANCE_DEGREES);
        return Math.abs(parallelogram.getAngle()-desiredAngle)<ParallelConstants.TOLERANCE_DEGREES;
    }

    public void end(boolean interrupted) {
        parallelogram.setPower(0);
    }
}

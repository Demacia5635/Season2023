package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class CalibrateParallelogram extends CommandBase {
    private Parallelogram parallelogram;
    private double testAngle;

    /**
     * Command's constructor.
     * @param parallelogram
     */
    public CalibrateParallelogram(Parallelogram parallelogram) {
        this.parallelogram = parallelogram;
        addRequirements(parallelogram);
    }


    @Override
    public void initialize() {
        testAngle = parallelogram.getAngle();
        parallelogram.setBrake();
        parallelogram.setPower(ParallelConstants.CALIBRATION_POWER);
    }


    @Override
    public boolean isFinished() {
        System.out.println("Activation Value: " + testAngle +  " Current Angle: " + parallelogram.getAngle() + " Desired Angle: " );
        return parallelogram.getDigitalInput();
    }

    @Override
    public void end(boolean interrupted) {
        parallelogram.setPower(0);
    }
}

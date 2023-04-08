package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class ResetCalibrate extends CommandBase {
    private Parallelogram parallelogram;

    /**
     * Command's constructor.
     * @param parallelogram
     */
    public ResetCalibrate(Parallelogram parallelogram) {
        this.parallelogram = parallelogram;
        addRequirements(parallelogram);
    }


    @Override
    public void initialize() {
        parallelogram.setBrake();
    }

    @Override
    public void execute() {
        parallelogram.setPower(ParallelConstants.BACKWARDS_CALIBRATION_POWER);
    }

    @Override
    public boolean isFinished() {
        return !parallelogram.getDigitalInput();
    }

    @Override
    public void end(boolean interrupted) {
        parallelogram.resetPosition();
        parallelogram.setPower(0);
    }
}
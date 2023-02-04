package frc.robot.commands.parallelogramCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class CalibrateParallelogram extends CommandBase {
    private Parallelogram parallelogram;

    public CalibrateParallelogram(Parallelogram parallelogram) {
        this.parallelogram = parallelogram;
    }

    @Override
    public void initialize() {
        parallelogram.setBrake();
    }

    @Override
    public void execute() {
        parallelogram.setPower(ParallelConstants.CALIBRATION_POWER);
    }

    @Override
    public boolean isFinished() {
        return parallelogram.getDigitalInput();
    }

    @Override
    public void end(boolean interrupted) {
        parallelogram.setPower(0);
        parallelogram.setCoast();
        parallelogram.resetPosition();
    }
}

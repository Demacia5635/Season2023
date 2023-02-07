package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class CalibrateParallelogram extends CommandBase {
    private Parallelogram parallelogram;

    /**
     * Command's constructor.
     * @param parallelogram
     */
    public CalibrateParallelogram(Parallelogram parallelogram) {
        this.parallelogram = parallelogram;
    }

    /**
     * Setting the motor's neutral mode as brake.
     */
    @Override
    public void initialize() {
        parallelogram.setBrake();
    }

    /**
     * Setting the motor's power.
     */
    @Override
    public void execute() {
        parallelogram.setPower(ParallelConstants.CALIBRATION_POWER);
    }

    /**
     * Checking if arm has reached the magnet (Digital Input = true).
     */
    @Override
    public boolean isFinished() {
        return parallelogram.getDigitalInput();
    }

    /**
     * Setting the motor's power to 0, neutral mode to coast and reseting the position/angle.
     */
    @Override
    public void end(boolean interrupted) {
        parallelogram.setPower(0);
        parallelogram.setCoast();
        parallelogram.resetPosition();
    }
}

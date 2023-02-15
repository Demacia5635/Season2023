package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

/**
 * A command that takes the arm to it's initial position.
 */
public class GoToBack extends CommandBase{

    Command end;
    /**
     * Command's constructor.
     * @param parallelogram
     */
    public GoToBack(Parallelogram parallelogram) {
        end = new GoToAngle(parallelogram, ParallelConstants.DIGITAL_INPUT_ANGLE);
    }

    @Override
    public void initialize() {
        end.schedule();
    }
    
}

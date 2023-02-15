package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.parallelogram.Parallelogram;

/**
 * Picks up a gamepiece from loading zone.
 */
public class PickUp extends CommandBase {
    private Command collect;

    /**
     * Constructs a new PickUp command.
     * @param parallelogram
     */
    public PickUp(Parallelogram parallelogram) {
        collect = new GoToHeight(parallelogram, Constants.LOADING_HEIGHT, true);
    }

    @Override
    public void initialize() {
        collect.schedule();
    }
    
}

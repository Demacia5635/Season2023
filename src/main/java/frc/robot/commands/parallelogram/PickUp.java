package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.parallelogram.Parallelogram;

/**
 * Picks up a gamepiece from loading zone.
 */
public class PickUp extends CommandBase {
    private Command collect;
    private Chassis chassis;

    /**
     * Constructs a new PickUp command.
     * @param parallelogram
     */
    public PickUp(Parallelogram parallelogram, Chassis chassis) {
        collect = new GoToHeight(parallelogram, Constants.LOADING_HEIGHT, true);
        this.chassis = chassis;
    }

    @Override
    public void end(boolean interrupted) {
        collect.schedule();
    }

    @Override
    public boolean isFinished() {
        return Constants.LOADING_ZONE.isInside(chassis.getPose().getTranslation()); //TODO: 
    }
    
}

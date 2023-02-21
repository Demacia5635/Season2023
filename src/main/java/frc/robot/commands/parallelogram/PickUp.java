package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.utils.ChassisUtils.Zone;
import frc.robot.subsystems.parallelogram.Parallelogram;

/**
 * Picks up a gamepiece from loading zone.
 */
public class PickUp extends CommandBase {
    private final Command collect;
    private final Chassis chassis;
    private boolean started;

    /**
     * Constructs a new PickUp command.
     * @param parallelogram
     */
    public PickUp(Parallelogram parallelogram, Chassis chassis) {
        collect = new GoToAngle(parallelogram, Constants.LOADING_ANGLE);
        this.chassis = chassis;
    }

    @Override
    public void initialize() {
        started = false;
    }

    @Override
    public void execute() {
        if (!started && Zone.fromRobotLocation(chassis.getPose().getTranslation())==Zone.LOADING_ZONE) {
            collect.schedule();
            started = true;
        }
 }

    @Override
    public boolean isFinished() {
        return collect.isFinished();
    }
    
}

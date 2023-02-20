package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.utils.ChassisUtils;
import frc.robot.subsystems.chassis.utils.ChassisUtils.Zone;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

/**
 * Picks up a gamepiece from loading zone.
 */
public class PickUp extends CommandBase {
    private Command collect;
    private Chassis chassis;
    private Parallelogram parallelogram;

    /**
     * Constructs a new PickUp command.
     * @param parallelogram
     */
    public PickUp(Parallelogram parallelogram, Chassis chassis) {
        collect = new GoToAngle(parallelogram, Constants.LOADING_ANGLE);
        this.chassis = chassis;
        this.parallelogram = parallelogram;
    }

    @Override
    public void execute() {
        if (ChassisUtils.Zone.fromRobotLocation(chassis.getPose().getTranslation())==Zone.LOADING_ZONE) {
            collect.schedule();
        }
 }

    @Override
    public boolean isFinished() {
        return Math.abs(parallelogram.getAngle() - Constants.LOADING_ANGLE) < ParallelConstants.TOLERANCE_DEGREES;
    }
    
}

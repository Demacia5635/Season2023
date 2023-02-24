package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;

/**
 * The command to open the gripper.
 */
public class Open extends CommandBase {
    private final Gripper gripper;

    /**
     * Creates a new Open.
     * 
     * @param gripper The gripper subsystem.
     */
    public Open(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void initialize() {
        gripper.open();
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }

    @Override
    public boolean isFinished() {
        return gripper.getCurrent() >= GripperConstants.OPEN_CURRENT;
    }
}
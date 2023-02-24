package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;

/**
 * The command to close the gripper.
 */
public class Close extends CommandBase {
    private final Gripper gripper;
    private final double maxCurrent;

    /**
     * Creates a new Close.
     * 
     * @param gripper The gripper subsystem.
     * @param current The current to stop at.
     */
    public Close(Gripper gripper, double current) {
        this.gripper = gripper;
        maxCurrent = current;
        addRequirements(gripper);
    }

    @Override
    public void initialize() {
        gripper.close();
    }

    @Override
    public boolean isFinished() {
        return gripper.getCurrent() >= maxCurrent;
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
}
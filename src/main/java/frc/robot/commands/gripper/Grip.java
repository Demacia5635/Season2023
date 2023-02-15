package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.gripper.Gripper;
/**
 * A command that grips a game piece.
 */
public class Grip extends CommandBase {
    private Gripper gripper;
    Command open;
    Command close;
    Command wait = new WaitCommand(0.5);
    /**
     * Creates new grip command.
     * @param gripper
     */
    public Grip(Gripper gripper) {
        this.gripper = gripper;
        open = gripper.getOpenCommand();
        close = gripper.getCloseCommand();
    }

    @Override
    public void initialize() {
        open.schedule();
        wait.schedule();
        close.schedule();
    }
}

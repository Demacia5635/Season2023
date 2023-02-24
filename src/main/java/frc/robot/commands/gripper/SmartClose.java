package frc.robot.commands.gripper;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.utils.GamePiece;

/**
 * The command to close the gripper.
 */
public class SmartClose extends CommandBase {
    private final Gripper gripper;
    private final Supplier<GamePiece> maxCurrentSupplier;

    /**
     * Creates a new SmartClose.
     * 
     * @param gripper           The gripper subsystem.
     * @param gamePieceSupplier The supplier for the current to stop at.
     */
    public SmartClose(Gripper gripper, Supplier<GamePiece> gamePieceSupplier) {
        this.gripper = gripper;
        this.maxCurrentSupplier = gamePieceSupplier;
    }

    @Override
    public void initialize() {
        gripper.close();
    }

    @Override
    public boolean isFinished() {
        return gripper.getCurrent() >= (maxCurrentSupplier.get() == GamePiece.CONE ? GripperConstants.CLOSE_CONE_CURRENT
                : GripperConstants.CLOSE_CUBE_CURRENT);
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
}

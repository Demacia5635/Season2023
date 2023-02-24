package frc.robot.subsystems.gripper;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.gripper.Close;
import frc.robot.commands.gripper.Open;
import frc.robot.commands.gripper.SmartClose;
import frc.robot.utils.GamePiece;

/**
 * The gripper subsystem.
 */
public class Gripper extends SubsystemBase {
    private final TalonSRX motor;

    private boolean isOpen;

    /**
     * Creates a new Gripper.
     */
    public Gripper() {
        motor = new TalonSRX(GripperConstants.MOTOR_PORT);
    }

    /**
     * Opens the gripper.
     */
    public void open() {
        motor.set(ControlMode.PercentOutput, GripperConstants.OPEN_PERCENT_OUTPUT);
        isOpen = true;
    }

    /**
     * Closes the gripper.
     */
    public void close() {
        motor.set(ControlMode.PercentOutput, GripperConstants.CLOSE_PERCENT_OUTPUT);
        isOpen = false;
    }

    /**
     * Stops the gripper.
     */
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Gets the current of the gripper.
     * 
     * @return The current of the gripper.
     */
    public double getCurrent() {
        return motor.getSupplyCurrent();
    }

    /**
     * Gets the open command.
     * 
     * @return The command that opens the gripper.
     */
    public CommandBase getOpenCommand() {
        return new Open(this);
    }

    /**
     * Gets the close cube command.
     * 
     * @return The command that closes the gripper on a cube.
     */
    public CommandBase getCloseCubeCommand() {
        return new Close(this, GripperConstants.CLOSE_CUBE_CURRENT);
    }

    /**
     * Gets the close cone command.
     * 
     * @return The command that closes the gripper on a cone.
     */
    public CommandBase getCloseConeCommand() {
        return new Close(this, GripperConstants.CLOSE_CONE_CURRENT);
    }

    public CommandBase getCloseCommand(Supplier<GamePiece> currentSupplier) {
        return new SmartClose(this, currentSupplier);
    }

    public CommandBase getSwitchPositionCommand(Supplier<GamePiece> gamePieceSupplier) {
        if (isOpen) {
            return getCloseCommand(gamePieceSupplier);
        }
        return getOpenCommand();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Current", this::getCurrent, null);

        SmartDashboard.putData("Open", getOpenCommand());
        SmartDashboard.putData("Close Cube", getCloseCubeCommand());
        SmartDashboard.putData("Close Cone", getCloseConeCommand());
    }
}
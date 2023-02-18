package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AlternativeGripper;

public class CalibrateGripper extends CommandBase {
  /** Creates a new AlternativeGripper. */
  private final AlternativeGripper alternativeGripper;

  public CalibrateGripper(AlternativeGripper alternativeGripper) {
    this.alternativeGripper = alternativeGripper;
    addRequirements(alternativeGripper);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alternativeGripper.setBreakMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //arbtriry power feel free to change
      alternativeGripper.setPowerToGripper(0.2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(alternativeGripper.getCurrent()>=Constants.AlternativeGripper.CLOSED_GRIPPER_CURRENT){
        return true;
    }

    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AlternativeGripper;

public class AlternativeGripperOpenner extends CommandBase {
  /** Creates a new AlternativeGripper. */
  private final AlternativeGripper alternativeGripper;
  private final XboxController controller;

  public AlternativeGripperOpenner(XboxController controller, AlternativeGripper alternativeGripper) {
    this.alternativeGripper = alternativeGripper;
    this.controller = controller;
    addRequirements(alternativeGripper);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean isOpen = alternativeGripper.isOpen();
    boolean isClose = alternativeGripper.isClosed();
    double power = controller.getLeftY();
    
    if (((isOpen || isClose) && Math.abs(power) > 0) || (Math.abs(power) < 0.1)) {
      power = 0;
    } 

      alternativeGripper.setPowerToGripper(power);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}

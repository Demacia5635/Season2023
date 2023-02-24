// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.GotoNodes;
import frc.robot.commands.chassis.GotoRamp;
import frc.robot.commands.chassis.LeaveCommunity;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.gripper.Gripper;

public class GenerateAutonomous extends CommandBase {
  private SendableChooser placeGamePeace;
  private SendableChooser leaveCommunity;
  private SendableChooser climb;
  private GotoNodes gotoNodes;
  private Gripper gripper;
  private Chassis chassis;
  private boolean finish;

  /** Creates a new GenerateAutonomous. */
  public GenerateAutonomous(GotoNodes gotoNodes, Gripper gripper, Chassis chassis) {
    this.gotoNodes = gotoNodes;
    this.gripper = gripper;
    this.chassis = chassis;
    finish = false;
    
    placeGamePeace = new SendableChooser<>();
        placeGamePeace.setDefaultOption("Top", true);
        placeGamePeace.addOption("Bottom", false);
        SmartDashboard.putData(placeGamePeace);

        leaveCommunity = new SendableChooser<>();
        leaveCommunity.setDefaultOption("Leave", true);
        leaveCommunity.addOption("Dont Leave", false);
        SmartDashboard.putData(leaveCommunity);

        climb = new SendableChooser<>();
        climb.setDefaultOption("Top", true);
        climb.addOption("Bottom", false);
        SmartDashboard.putData(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  public SequentialCommandGroup getAutonomous(){
    SequentialCommandGroup autonomous = new SequentialCommandGroup(placeGamePeace.getSelected().equals(true) ? gotoNodes.andThen(gripper.getOpenCommand()) 
        : new InstantCommand(()-> System.out.println("didnt place Game Peace")).andThen(leaveCommunity.getSelected().equals(true)?
         new LeaveCommunity(chassis) : new InstantCommand(()-> System.out.println("didnt leave"))).andThen(climb.getSelected().equals(true)?
          new GotoRamp(chassis) :new InstantCommand(()->System.out.println("didnt climb"))));
    finish = true;
    return autonomous;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}

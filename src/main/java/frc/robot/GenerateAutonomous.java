// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
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

/** Add your docs here. */
public class GenerateAutonomous {
  private SendableChooser placeGamePeace;
  private SendableChooser exitCommunity;
  private SendableChooser climb;
  private GotoNodes gotoNodes;
  private LeaveCommunity leaveCommunity;
  private Gripper gripper;
  private Chassis chassis;
  private boolean finish;

  /** Creates a new GenerateAutonomous. */
  public GenerateAutonomous(GotoNodes gotoNodes, LeaveCommunity leaveCommunity, Gripper gripper, Chassis chassis) {
    this.gotoNodes = gotoNodes;
    this.leaveCommunity = leaveCommunity;
    this.gripper = gripper;
    this.chassis = chassis;
    finish = false;
    
    placeGamePeace = new SendableChooser<Boolean>();
        placeGamePeace.setDefaultOption("Top", true);
        placeGamePeace.addOption("Bottom", false);
        SmartDashboard.putData(placeGamePeace);

        exitCommunity = new SendableChooser<Boolean>();
        exitCommunity.setDefaultOption("Leave", true);
        exitCommunity.addOption("Dont Leave", false);
        SmartDashboard.putData(leaveCommunity);

        climb = new SendableChooser<Boolean>();
        climb.setDefaultOption("Top", true);
        climb.addOption("Bottom", false);
        SmartDashboard.putData(climb);
  }

  public SequentialCommandGroup getAutonomous(){
    SequentialCommandGroup autonomous = new SequentialCommandGroup(placeGamePeace.getSelected().equals(true) ? gotoNodes.andThen(gripper.getOpenCommand()) 
        : new InstantCommand(()-> System.out.println("didnt place Game Peace")).andThen(exitCommunity.getSelected().equals(true)?
         new LeaveCommunity(chassis) : new InstantCommand(()-> System.out.println("didnt leave"))).andThen(climb.getSelected().equals(true)?
          new GotoRamp(chassis) :new InstantCommand(()->System.out.println("didnt climb"))));
    finish = true;
    return autonomous;
  }
}



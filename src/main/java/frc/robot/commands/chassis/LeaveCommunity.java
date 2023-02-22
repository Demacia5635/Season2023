// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import com.ctre.phoenixpro.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.utils.ChassisUtils.Zone;
import frc.robot.utils.UtilsGeneral;
import frc.robot.subsystems.chassis.utils.TrajectoryGenerator;


public class LeaveCommunity extends CommandBase {
  private final Chassis chassis;
  private Command command;
  private final XboxController controller;
  public static enum TopOrBottom{
    TOP, BOTTOM;
  }

  private TopOrBottom topOrBottom;
  /** Creates a new LeaveCommunity. */
  public LeaveCommunity(Chassis chassis, TopOrBottom topOrBottom, XboxController controller) {
    this.controller = controller;
    this.chassis = chassis;
    addRequirements(chassis);
    this.topOrBottom = topOrBottom;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command = new InstantCommand();
    TrajectoryGenerator generator = new TrajectoryGenerator(Alliance.Blue);

    Zone zone = Zone.fromRobotLocation(chassis.getPose().getTranslation());
    if(zone == Zone.COMMUNITY_BOTTOM || zone == Zone.COMMUNITY_MIDDLE || zone == Zone.COMMUNITY_TOP){
      if(topOrBottom == TopOrBottom.TOP){
        generator.add(new Pose2d(new Translation2d(2.06,4.02), new Rotation2d()), Rotation2d.fromDegrees(70.79));
        generator.add(new Pose2d(new Translation2d(4.28,4.89), new Rotation2d()), Rotation2d.fromDegrees(-3.47));
      }else if(topOrBottom == TopOrBottom.BOTTOM){
        generator.add(new Pose2d(new Translation2d(2.23, 0.65), new Rotation2d()), Rotation2d.fromDegrees(5.44));
        generator.add(new Pose2d(new Translation2d(5.57, 0.70), new Rotation2d()), Rotation2d.fromDegrees(1.59));
      }
    }

    command = chassis.createPathFollowingCommand(false, generator.generate(chassis.getPose()));
    command.schedule();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
    command.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return CommandScheduler.getInstance().requiring(chassis) != command || UtilsGeneral.hasInput(controller);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.utils.ChassisUtils.Zone;
import frc.robot.subsystems.chassis.utils.TrajectoryGenerator;

public class LeaveCommunity extends CommandBase {
    private final Chassis chassis;
    private Command command;

    public static enum TopOrBottom {
        TOP, BOTTOM;
    }

    public static enum ExitOrRamp {
        TO_EXIT , TO_RAMP;
    }


    private SendableChooser<TopOrBottom> chooserTopOrBottom;
    private SendableChooser<ExitOrRamp> chooserExitOrRamp;


    /** Creates a new LeaveCommunity. */
    public LeaveCommunity(Chassis chassis) {
        this.chassis = chassis;
        chooserTopOrBottom = new SendableChooser<>();
        chooserTopOrBottom.setDefaultOption("Top", TopOrBottom.TOP);
        chooserTopOrBottom.addOption("Bottom", TopOrBottom.BOTTOM);
        SmartDashboard.putData("Leave topOtBottom", chooserTopOrBottom);

        chooserExitOrRamp = new SendableChooser<>();
        chooserExitOrRamp.setDefaultOption("To Exit", ExitOrRamp.TO_EXIT);
        chooserExitOrRamp.addOption("To Ramp", ExitOrRamp.TO_RAMP);
        SmartDashboard.putData("Leave ExitOrRamp",chooserExitOrRamp);

        addRequirements(chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        command = new InstantCommand();
        TrajectoryGenerator generator1 = new TrajectoryGenerator(Alliance.Blue);
        TrajectoryGenerator generator2 = new TrajectoryGenerator(Alliance.Blue);

        Zone zone = Zone.fromRobotLocation(chassis.getPose().getTranslation());
        if (chooserTopOrBottom.getSelected() == TopOrBottom.TOP) {
            switch (zone) {
                case COMMUNITY_MIDDLE:
                case COMMUNITY_BOTTOM:
                    generator1.add(new Pose2d(new Translation2d(2.2, 4.7), Rotation2d.fromDegrees(180)));
                    generator2.add(new Pose2d(new Translation2d(2.2, 4.7), Rotation2d.fromDegrees(180)));
                case COMMUNITY_TOP:
                    generator2.add(new Pose2d(new Translation2d(5.8, 4.7), Rotation2d.fromDegrees(180)));
                default:
                    break;
            }
            if (chooserExitOrRamp.getSelected() == ExitOrRamp.TO_RAMP) {
                generator2.add(new Pose2d(new Translation2d(6.55, 3.15), Rotation2d.fromDegrees(235)));
            } else {
                generator2.add(new Pose2d(new Translation2d(5.8, 4.7)   , Rotation2d.fromDegrees(0)));
            }
        } else {
            switch (zone) {
                case COMMUNITY_MIDDLE:
                case COMMUNITY_TOP:
                    generator1.add(new Pose2d(new Translation2d(2.2, 0.8), Rotation2d.fromDegrees(180)));
                    generator2.add(new Pose2d(new Translation2d(2.2, 0.8), Rotation2d.fromDegrees(180)));

                case COMMUNITY_BOTTOM:
                    generator2.add(new Pose2d(new Translation2d(5.8, 0.8), Rotation2d.fromDegrees(180)));
                default:
                    break;
            }
            if(chooserExitOrRamp.getSelected() == ExitOrRamp.TO_RAMP){
                generator2.add(new Pose2d(new Translation2d(6.55, 2.3), Rotation2d.fromDegrees(235)));
            } else {
                generator2.add(new Pose2d(new Translation2d(5.8, 0.8), Rotation2d.fromDegrees(0)));
            }
        }
        if (generator1.length() == 0)
            command = chassis.createPathFollowingCommand(false, generator2.generate(chassis.getPose()));
        else
            command = chassis.createPathFollowingCommand(false, generator1.generate(chassis.getPose())).andThen(chassis.createPathFollowingCommand(false, generator2.generate()));
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
        chassis.stop();
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
    
}

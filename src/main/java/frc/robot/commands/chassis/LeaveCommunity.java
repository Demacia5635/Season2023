
package frc.robot.commands.chassis;

import com.pathplanner.lib.PathConstraints;

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
import frc.robot.subsystems.chassis.ChassisConstants;
import frc.robot.subsystems.chassis.utils.ChassisUtils.Zone;
import frc.robot.subsystems.chassis.utils.TrajectoryGenerator;

public class LeaveCommunity extends CommandBase {
    private final Chassis chassis;
    private Command command;

    public static enum StartPlacement {
        TOP, BOTTOM, MIDDLE;
    }

    public static enum ExitOrRamp {
        TO_EXIT , TO_RAMP;
    }


    private SendableChooser<StartPlacement> chooserStartPlacement;
    private SendableChooser<ExitOrRamp> chooserExitOrRamp;


    /** Creates a new LeaveCommunity. */
    public LeaveCommunity(Chassis chassis) {
        this.chassis = chassis;
        chooserStartPlacement = new SendableChooser<>();
        chooserStartPlacement.setDefaultOption("Top", StartPlacement.TOP);
        chooserStartPlacement.addOption("Bottom", StartPlacement.BOTTOM);
        chooserStartPlacement.addOption("Middle", StartPlacement.MIDDLE);
        SmartDashboard.putData("Leave topOtBottom", chooserStartPlacement);

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
        PathConstraints pathConstraints = ChassisConstants.PATH_CONSTRAINTS;
        TrajectoryGenerator generator1 = new TrajectoryGenerator(Alliance.Blue);
        TrajectoryGenerator generator2 = new TrajectoryGenerator(Alliance.Blue);
        Zone zone = Zone.fromRobotLocation(chassis.getPose().getTranslation());


        if (chooserStartPlacement.getSelected() == StartPlacement.TOP) {
            switch (zone) {
                case COMMUNITY_MIDDLE:
                case COMMUNITY_BOTTOM:
                case COMMUNITY_TOP:
                    generator1.add(new Pose2d(new Translation2d(2.2, 4.75), Rotation2d.fromDegrees(180)));
                    generator2.add(new Pose2d(new Translation2d(2.2, 4.75), Rotation2d.fromDegrees(180)));
                    generator2.add(new Pose2d(new Translation2d(5.8, 4.75), Rotation2d.fromDegrees(180)));
                default:
                    break;
            }

            if (chooserExitOrRamp.getSelected() == ExitOrRamp.TO_RAMP) {
                generator2.add(new Pose2d(new Translation2d(6.55, 3.05), Rotation2d.fromDegrees(180)));
            } 
            else {
                generator2.add(new Pose2d(new Translation2d(5.8, 4.7), Rotation2d.fromDegrees(180)));
            }
        } 
        
        else if (chooserStartPlacement.getSelected() == StartPlacement.BOTTOM){
            switch (zone) {
                case COMMUNITY_MIDDLE:
                case COMMUNITY_TOP:
                case COMMUNITY_BOTTOM:
                    generator1.add(new Pose2d(new Translation2d(2.2, 0.6), Rotation2d.fromDegrees(180)));
                    generator2.add(new Pose2d(new Translation2d(2.2, 0.6), Rotation2d.fromDegrees(180)));
                    generator2.add(new Pose2d(new Translation2d(5.8, 0.6), Rotation2d.fromDegrees(180)));
                default:
                    break;
            }

            if(chooserExitOrRamp.getSelected() == ExitOrRamp.TO_RAMP){
                generator2.add(new Pose2d(new Translation2d(6.55, 2.3), Rotation2d.fromDegrees(180)));
            }
            else {
                generator2.add(new Pose2d(new Translation2d(5.8, 0.6), Rotation2d.fromDegrees(180)));
            }
        } 
        
        else {
            switch (zone) {
                case COMMUNITY_MIDDLE:
                case COMMUNITY_TOP:
                case COMMUNITY_BOTTOM:
                    generator1.add(new Pose2d(new Translation2d(2.2, 2.75), Rotation2d.fromDegrees(180))); //TODO: change values to fit "before ramp", "on ramp" and "after ramp"
                    generator2.add(new Pose2d(new Translation2d(2.2, 2.75), Rotation2d.fromDegrees(180)));
                    generator2.add(new Pose2d(new Translation2d(5.75, 2.75), Rotation2d.fromDegrees(180)));
                default:
                    System.out.println("here");
                    break;
            }
            pathConstraints = ChassisConstants.PATH_CONSTANTS_AUTO_MIDDLE;

        }

                                                                                                                                                                                                
        if (generator1.length() == 0)
            command = chassis.createPathFollowingCommand(false, generator2.generate(chassis.getPose()));
        else
            command = chassis.createPathFollowingCommand(false, generator1.generate(chassis.getPose()))
            .andThen(chassis.createPathFollowingCommand(false, pathConstraints, generator2.generate()));
            System.out.println("Initializing LC commandvfro middle");
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
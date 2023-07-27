package frc.robot.commands.chassis;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;
import frc.robot.subsystems.chassis.utils.ChassisCommands;
import frc.robot.subsystems.chassis.utils.TrajectoryGenerator;

/**
 * Goes to the ramp, assuming leave community was called before this.
 */
public class GotoRamp extends CommandBase {
    private final Chassis chassis;
    private Command command;

    /** Creates a new GotoRamp. */
    public GotoRamp(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        TrajectoryGenerator generator = new TrajectoryGenerator(Alliance.Blue);
        generator.add(new Pose2d(new Translation2d(3.91, 2.75), Rotation2d.fromDegrees(235)),
                Rotation2d.fromDegrees(180));

        command = ChassisCommands.createPathFollowingCommand(new PathConstraints(1.5, ChassisConstants.MAX_AUTO_ACCELERATION),
                generator.generate(chassis.getPose()));
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
        chassis.setRampPosition();
    }
}

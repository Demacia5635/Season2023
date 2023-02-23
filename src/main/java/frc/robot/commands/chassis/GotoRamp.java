package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.chassis.Chassis;
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
    }

    @Override
    public void initialize() {
        TrajectoryGenerator generator = new TrajectoryGenerator(Alliance.Blue);
        generator.add(new Pose2d(new Translation2d(5.6, 2.75), Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180));
        generator.add(new Pose2d(new Translation2d(3.91, 2.75), Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180));

        command = chassis.createPathFollowingCommand(generator.generate(chassis.getPose()));
        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return CommandScheduler.getInstance().requiring(chassis) != command;
    }

    @Override
    public void end(boolean interrupted) {
        command.cancel();
        chassis.stop();
    }
}

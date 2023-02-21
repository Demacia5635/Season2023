package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.utils.TrajectoryGenerator;
import frc.robot.utils.UtilsGeneral;
import frc.robot.utils.UtilsGeneral.Zone;

/**
 * Drives the robot semi autonomously to the loading zone.
 */
public class GotoLoadingZone extends CommandBase {

    private final Chassis chassis;
    private final XboxController controller;
    private final Command onEntry;
    private boolean entered;

    private Command command;

    /**
     * Constructs a new GotoLoadingZone command.
     * 
     * @param chassis    The chassis subsystem
     * @param controller The controller to check for input
     */
    public GotoLoadingZone(Chassis chassis, XboxController controller, Command onEntry) {
        this.chassis = chassis;
        this.controller = controller;
        this.onEntry = onEntry;
    }

    public GotoLoadingZone(Chassis chassis, XboxController controller) {
        this(chassis, controller, new InstantCommand());
    }

    @Override
    public void initialize() {
        entered = false;
        TrajectoryGenerator generator = new TrajectoryGenerator(Alliance.Blue);

        Zone zone = Zone.fromRobotLocation(chassis.getPose().getTranslation());

        if (zone == Zone.COMMUNITY_BOTTOM || zone == Zone.COMMUNITY_ENTRANCE_BOTTOM) {
            generator.add(new Pose2d(new Translation2d(5.3, 0.76), new Rotation2d()),
                    new Rotation2d());
            generator.add(new Pose2d(new Translation2d(11.11, 6.25), new Rotation2d()),
                    new Rotation2d());
            generator.add(new Pose2d(new Translation2d(15.23, 6.25), new Rotation2d()),
                    new Rotation2d());
        } else {
            switch (zone) {
                case COMMUNITY_MIDDLE:
                    generator.add(new Pose2d(new Translation2d(2.17, 4.74), new Rotation2d()),
                            new Rotation2d());
                case COMMUNITY_TOP:
                case COMMUNITY_ENTRANCE_TOP:
                    generator.add(new Pose2d(new Translation2d(5.57, 4.9), new Rotation2d()),
                            new Rotation2d());
                case OPEN_AREA:
                    generator.add(new Pose2d(new Translation2d(11.11, 6.25), new Rotation2d()),
                            new Rotation2d());
                case LOADING_ZONE:
                default:
                    generator.add(new Pose2d(new Translation2d(15.23, 6.25), new Rotation2d()),
                            new Rotation2d());
            }
        }

        command = chassis.createPathFollowingCommand(generator.generate(chassis.getPose()));

        command.schedule();
    }

    @Override
    public void execute() {
        if (!entered && Zone.fromRobotLocation(chassis.getPose().getTranslation()) == Zone.LOADING_ZONE) {
            onEntry.schedule();
            entered = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (CommandScheduler.getInstance().requiring(chassis) != command && onEntry.isFinished()) || UtilsGeneral.hasInput(controller);
    }

    @Override
    public void end(boolean interrupted) {
        command.cancel();
        chassis.stop();
    }
}

package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.utils.TrajectoryGenerator;
import frc.robot.utils.UtilsGeneral;
import frc.robot.utils.UtilsGeneral.Zone;

/**
 * Drives the robot semi autonomously to the loading zone.
 */
public class GotoLoadingZone extends CommandBase {

    private final Chassis chassis;
    private final Command onEntry;
    private boolean entered;

    private Command command;
    private Position position;

    private boolean commandEnded;
    private boolean onEntryEnded;

    /**
     * Constructs a new GotoLoadingZone command.
     * 
     * @param chassis    The chassis subsystem
     * @param controller The controller to check for input
     */
    public GotoLoadingZone(Chassis chassis, CommandXboxController secondary, Command onEntry, Position position) {
        this.chassis = chassis;
        this.onEntry = onEntry;
        this.position = position;

        secondary.a().onTrue(new InstantCommand(() -> {
            if (this.position == Position.TOP) {
                this.position = Position.BOTTOM;

            } else {
                this.position = Position.TOP;
            }
        }).ignoringDisable(true));

        addRequirements(onEntry.getRequirements().toArray(Subsystem[]::new));
        addRequirements(chassis);
        SmartDashboard.putData(this);
    }

    public GotoLoadingZone(Chassis chassis, CommandXboxController secondary, Position position) {
        this(chassis, secondary, new InstantCommand(), position);
    }
    
    public GotoLoadingZone(Chassis chassis, CommandXboxController secondary, Command onEntry) {
        this(chassis, secondary, onEntry, Position.BOTTOM);
    }

    public GotoLoadingZone(Chassis chassis, CommandXboxController secondary) {
        this(chassis, secondary, new InstantCommand(), Position.BOTTOM);
    }

    public static enum Position {
        TOP, BOTTOM
    }

    @Override
    public void initialize() {
        chassis.forceUseVision();
        onEntryEnded = false;
        commandEnded = false;
        entered = false;
        TrajectoryGenerator generator = new TrajectoryGenerator(Alliance.Blue);
        double endY = position == Position.TOP ? 7.5 : 6.25;

        Zone zone = Zone.fromRobotLocation(chassis.getPose().getTranslation());

        if (zone == Zone.COMMUNITY_BOTTOM || zone == Zone.COMMUNITY_ENTRANCE_BOTTOM) {
            generator.add(new Pose2d(new Translation2d(5.3, 0.76), new Rotation2d()),
                    new Rotation2d());
            generator.add(new Pose2d(new Translation2d(11.11, endY), new Rotation2d()),
                    new Rotation2d());
            generator.add(new Pose2d(new Translation2d(15.08, endY), new Rotation2d()),
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
                    generator.add(new Pose2d(new Translation2d(11.11, endY), new Rotation2d()),
                            new Rotation2d());
                case LOADING_ZONE:
                default:
                    generator.add(new Pose2d(new Translation2d(15.08, endY), new Rotation2d()),
                            new Rotation2d());
            }
        }

        command = chassis.createPathFollowingCommand(generator.generate(chassis.getPose()));

        command.initialize();
    }

    @Override
    public void execute() {
        if (!entered && Zone.fromRobotLocation(chassis.getPose().getTranslation()) == Zone.LOADING_ZONE) {
            onEntry.initialize();
            entered = true;
        } else if (entered && !onEntryEnded) {
            onEntry.execute();
        }
        if (!commandEnded)
            command.execute();
    }

    @Override
    public boolean isFinished() {
        if (!onEntryEnded) {
            onEntryEnded = onEntry.isFinished();
        }
        if (!commandEnded) {
            commandEnded = command.isFinished();
        }
        return commandEnded && onEntryEnded;
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
        onEntry.end(interrupted);
        chassis.stop();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Position", () -> {
            if (UtilsGeneral.isRedAlliance()) {
                return position == Position.TOP ? "Right" : "Left";
            } else {
                return position == Position.TOP ? "Left" : "Right";
            }
        }, null);
    }
}

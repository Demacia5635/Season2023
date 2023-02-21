package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.utils.TrajectoryGenerator;
import frc.robot.utils.UtilsGeneral;

/**
 * This command is used to go to the nodes on the field from the community.
 */
public class GotoNodes extends CommandBase {

    private static final Translation2d[][] NODES = {
            { new Translation2d(1.38, 0.51), new Translation2d(1.38, 1.07), new Translation2d(1.38, 1.63) },
            { new Translation2d(1.38, 2.19), new Translation2d(1.38, 2.75), new Translation2d(1.38, 3.31) },
            { new Translation2d(1.38, 3.87), new Translation2d(1.38, 4.43), new Translation2d(1.38, 4.99) }
    }; // All relative to blue alliance

    /** Distance the robot should be from the node of the cube */
    private static final double DISTANCE_CUBE = 0.67;
    /** Distance the robot should be from the node of the cone */
    private static final double DISTANCE_CONE = 0.67;

    /**
     * The position of the robot on the grid.
     */
    public static enum Position {
        BOTTOM, MIDDLE, TOP;

        public int getValue() {
            switch (this) {
                case TOP:
                    return 2;
                case MIDDLE:
                    return 1;
                case BOTTOM:
                    return 0;
                default:
                    return -1;
            }
        }

        public static Position fromAllianceRelative(Position position) {
            if (UtilsGeneral.isRedAlliance()) {
                switch (position) {
                    case BOTTOM:
                        return TOP;
                    case MIDDLE:
                        return MIDDLE;
                    default:
                        return BOTTOM;
                }
            }
            return position;
        }
    }

    private final Chassis chassis;
    private final CommandXboxController controller;
    private Command command;

    private Position gridPosition;

    private Position nodePosition;

    private boolean isScheduled;
    private final Command onPosition;

    /**
     * Constructor for the GotoNodes command.
     * 
     * @param chassis
     * @param controller
     */
    public GotoNodes(Chassis chassis, CommandXboxController controller, Command onPosition) {
        this.chassis = chassis;
        this.controller = controller;
        this.onPosition = onPosition;
        command = new InstantCommand();
        isScheduled = false;
    }

    /**
     * Constructor for the GotoNodes command.
     * 
     * @param chassis
     * @param controller
     */
    public GotoNodes(Chassis chassis, CommandXboxController controller) {
        nodePosition = Position.BOTTOM;
        gridPosition = Position.BOTTOM;
        controller.x().and(controller.povLeft()).onTrue(new InstantCommand(()->doChangeTarget(Position.BOTTOM, Position.TOP)).ignoringDisable(true));
        controller.x().and(controller.povUp()).onTrue(new InstantCommand(()->doChangeTarget(Position.BOTTOM, Position.MIDDLE)).ignoringDisable(true));
        controller.x().and(controller.povRight()).onTrue(new InstantCommand(()->doChangeTarget(Position.BOTTOM, Position.BOTTOM)).ignoringDisable(true));
        controller.y().and(controller.povLeft()).onTrue(new InstantCommand(()->doChangeTarget(Position.MIDDLE, Position.TOP)).ignoringDisable(true));
        controller.y().and(controller.povUp()).onTrue(new InstantCommand(()->doChangeTarget(Position.MIDDLE, Position.MIDDLE)).ignoringDisable(true));
        controller.y().and(controller.povRight()).onTrue(new InstantCommand(()->doChangeTarget(Position.MIDDLE, Position.BOTTOM)).ignoringDisable(true));
        controller.b().and(controller.povLeft()).onTrue(new InstantCommand(()->doChangeTarget(Position.TOP, Position.TOP)).ignoringDisable(true));
        controller.b().and(controller.povUp()).onTrue(new InstantCommand(()->doChangeTarget(Position.TOP, Position.MIDDLE)).ignoringDisable(true));
        controller.b().and(controller.povRight()).onTrue(new InstantCommand(()->doChangeTarget(Position.TOP, Position.TOP)).ignoringDisable(true));
        
        this.chassis = chassis;
        this.controller = controller;
        onPosition = new InstantCommand();
        command = new InstantCommand();
        isScheduled = false;
        SmartDashboard.putData(this);

    }

    private void doChangeTarget(Position grid, Position node){
        changeTarget(Position.fromAllianceRelative(grid), Position.fromAllianceRelative(node));
    }
    
    /**
     * Initialize the command.
     */
    private void initCommand() {
        Translation2d node = NODES[gridPosition.getValue()][nodePosition.getValue()];
        if (nodePosition == Position.MIDDLE) {
            node = node.plus(new Translation2d(DISTANCE_CUBE, 0));
        } else {
            node = node.plus(new Translation2d(DISTANCE_CONE, 0));
        }

        TrajectoryGenerator generator = new TrajectoryGenerator(Alliance.Blue);

        generator.add(new Pose2d(node, Rotation2d.fromDegrees(180)));

        command = chassis.createPathFollowingCommand(onPosition.asProxy(), generator.generate(chassis.getPose()));
    }

    @Override
    public void initialize() {
        isScheduled = true;
    }

    /**
     * Changes the target of the command to the target selected in the Smart
     * Dashboard.
     */
    private void changeTarget(Position grid, Position node) {
        gridPosition = grid;
        nodePosition = node;

        if (command.isScheduled()) {
            command.cancel();
        }
        initCommand();
        if (isScheduled)
            command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        command.cancel();
        chassis.stop();
        isScheduled = false;
    }

    @Override
    public boolean isFinished() {
        return UtilsGeneral.hasInput(controller.getHID()) || CommandScheduler.getInstance().requiring(chassis) != command;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Grid selected pos", ()->{
            switch (gridPosition) {
                case BOTTOM:
                    return "3";
                    
                case MIDDLE:
                    return "2";
                
                case TOP:
                    return "1";
            
                default:
                    return "NON SELECTED";
            }
        }, null);

        builder.addStringProperty("Node selected pos", ()->{
            switch (nodePosition) {
                case BOTTOM:
                    return "C";
                    
                case MIDDLE:
                    return "B";
                
                case TOP:
                    return "A";
            
                default:
                    return "NON SELECTED";
            }
        }, null);
    }
}

package frc.robot.commands.chassis;

import java.util.Random;
import java.util.function.Supplier;

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

/**
 * This command is used to go to the nodes on the field from the community.
 */
public class GotoNodes extends CommandBase {
    //TODO : DEKET RANDOMIZED VALUES
    private static final Translation2d[][] NODES = {
            { new Translation2d(1.38 + getRandomizedPos(0.2, 2), 0.51 + getRandomizedPos(0.2, 2)), new Translation2d(1.38 + getRandomizedPos(0.2, 2), 1.07 + getRandomizedPos(0.2, 2)), new Translation2d(1.38 + getRandomizedPos(0.2, 2), 1.63 + getRandomizedPos(0.2, 2)) },
            { new Translation2d(1.38 + getRandomizedPos(0.2, 2), 2.19 + getRandomizedPos(0.2, 2)), new Translation2d(1.38 + getRandomizedPos(0.2, 2), 2.75 + getRandomizedPos(0.2, 2)), new Translation2d(1.38 + getRandomizedPos(0.2, 2), 3.31 + getRandomizedPos(0.2, 2)) },
            { new Translation2d(1.38 + getRandomizedPos(0.2, 2) , 3.87 + getRandomizedPos(0.2, 2)), new Translation2d(1.38 + getRandomizedPos(0.2, 2), 4.43 + getRandomizedPos(0.2, 2)), new Translation2d(1.38 + getRandomizedPos(0.2, 2), 4.99 + getRandomizedPos(0.2, 2)) }
    }; // All relative to blue alliance

 //TODO: DELETE THE RANDIMIZER
 private static double getRandomizedPos(double error, int options){
    return new Random().nextInt(options) != 0 ? 0 : error;
}

    /** Distance the robot should be from the node of the cube */
    private static final double DISTANCE_CUBE = 0.59;
    /** Distance the robot should be from the node of the cone */
    private static final double DISTANCE_CONE = 0.59;

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
    private Command command;

    private Position gridPosition;

    private Position nodePosition;

    private boolean isScheduled;
    private final Supplier<Command> onPosition;

    /**
     * Constructor for the GotoNodes command.
     * 
     * @param chassis
     * @param secondary
     */
    public GotoNodes(Chassis chassis, CommandXboxController secondary, Supplier<Command> onPosition) {
        nodePosition = Position.BOTTOM;
        gridPosition = Position.BOTTOM;
        secondary.x().and(secondary.povLeft()).onTrue(new InstantCommand(()->doChangeTarget(Position.TOP, Position.TOP)).ignoringDisable(true));
        secondary.x().and(secondary.povUp()).onTrue(new InstantCommand(()->doChangeTarget(Position.TOP, Position.MIDDLE)).ignoringDisable(true));
        secondary.x().and(secondary.povRight()).onTrue(new InstantCommand(()->doChangeTarget(Position.TOP, Position.BOTTOM)).ignoringDisable(true));
        secondary.y().and(secondary.povLeft()).onTrue(new InstantCommand(()->doChangeTarget(Position.MIDDLE, Position.TOP)).ignoringDisable(true));
        secondary.y().and(secondary.povUp()).onTrue(new InstantCommand(()->doChangeTarget(Position.MIDDLE, Position.MIDDLE)).ignoringDisable(true));
        secondary.y().and(secondary.povRight()).onTrue(new InstantCommand(()->doChangeTarget(Position.MIDDLE, Position.BOTTOM)).ignoringDisable(true));
        secondary.b().and(secondary.povLeft()).onTrue(new InstantCommand(()->doChangeTarget(Position.BOTTOM, Position.TOP)).ignoringDisable(true));
        secondary.b().and(secondary.povUp()).onTrue(new InstantCommand(()->doChangeTarget(Position.BOTTOM, Position.MIDDLE)).ignoringDisable(true));
        secondary.b().and(secondary.povRight()).onTrue(new InstantCommand(()->doChangeTarget(Position.BOTTOM, Position.BOTTOM)).ignoringDisable(true));
        this.chassis = chassis;
        this.onPosition = onPosition;
        command = new InstantCommand();
        isScheduled = false;

        addRequirements(chassis);
        addRequirements(onPosition.get().getRequirements().toArray(Subsystem[]::new));
        SmartDashboard.putData(this);
    }

    /**
     * Constructor for the GotoNodes command.
     * 
     * @param chassis
     * @param controller
     */
    public GotoNodes(Chassis chassis, CommandXboxController secondary) {   
        this(chassis, secondary, () -> new InstantCommand());
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

        command = chassis.createPathFollowingCommand(onPosition.get(), generator.generate(chassis.getPose()));
    }

    @Override
    public void initialize() {
        isScheduled = true;
        changeTarget(gridPosition, nodePosition);
    }

    /**
     * Changes the target of the command to the target selected in the Smart
     * Dashboard.
     */
    private void changeTarget(Position grid, Position node) {
        gridPosition = grid;
        nodePosition = node;

        command.end(true);
        initCommand();
        if (isScheduled)
            command.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
        chassis.stop();
        isScheduled = false;
        System.out.println("goToCommunity Ended");
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
    
    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Grid selected pos", ()->{
            Position t = Position.fromAllianceRelative(gridPosition);
            switch (t) {
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
            Position t = Position.fromAllianceRelative(nodePosition);
            switch (t) {
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

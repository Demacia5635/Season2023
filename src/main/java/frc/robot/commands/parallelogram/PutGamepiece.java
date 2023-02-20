package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.utils.ChassisUtils;
import frc.robot.subsystems.chassis.utils.ChassisUtils.Zone;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

/**
 * A command that puts a specified gamepiece in its node.
 */
public class PutGamepiece extends CommandBase{

    private double angle;
    private Command goToAngle;
    private Parallelogram parallelogram;

    public static enum GamePiece {
        CONE, CUBE;
        public int getValue() {
            switch (this) {
                case CONE:
                    return 1;
                case CUBE:
                    return 0;
                default:
                    return -1;
            }
        }
    }

    private final SendableChooser<GamePiece> gamepieceChooser;
    private GamePiece gamepiece;
    private Chassis chassis;
    /**
     * Constructs the PutGamePiece command.
     * @param parallelogram
     */
    public PutGamepiece(Parallelogram parallelogram, Chassis chassis) {
        this.parallelogram = parallelogram;
        this.chassis = chassis;
        gamepieceChooser = new SendableChooser<>();
        innitChooser();
        
    }
    /**
     * Initializes gamepiece chooser.
     */
    private void innitChooser() {
        gamepieceChooser.setDefaultOption("Cone", GamePiece.CONE);
        gamepieceChooser.addOption("Cube", GamePiece.CUBE);
        SmartDashboard.putData("Gamepiece Chooser", gamepieceChooser);

    }

    @Override
    public void initialize() {

        gamepiece = gamepieceChooser.getSelected();

        if (gamepiece.getValue()==1) {
            this.angle = Constants.CONE_ANGLE;
        }
        else {
            this.angle = Constants.CUBE_ANGLE;
        }

        goToAngle = new GoToAngle(parallelogram, angle);
    }

    @Override
    public void execute() {
        if (ChassisUtils.Zone.fromRobotLocation(chassis.getPose().getTranslation())==Zone.COMMUNITY_BOTTOM||
        ChassisUtils.Zone.fromRobotLocation(chassis.getPose().getTranslation())==Zone.COMMUNITY_TOP||
        ChassisUtils.Zone.fromRobotLocation(chassis.getPose().getTranslation())==Zone.COMMUNITY_MIDDLE) {
            goToAngle.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(parallelogram.getAngle() - angle) < ParallelConstants.TOLERANCE_DEGREES;
    }
    
}

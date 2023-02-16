package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.parallelogram.Parallelogram;

/**
 * A command that puts a specified gamepiece in its node.
 */
public class PutGamepiece extends CommandBase{

    private double angle;
    private Command goToHeight;
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
    /**
     * Constructs the PutGamePiece command.
     * @param parallelogram
     */
    public PutGamepiece(Parallelogram parallelogram) {
        this.parallelogram = parallelogram;
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

        goToHeight = new GoToAngle(parallelogram, angle);
        goToHeight.schedule();
    }
    
}

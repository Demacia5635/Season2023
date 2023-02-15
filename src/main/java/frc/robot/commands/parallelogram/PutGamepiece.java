package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.parallelogram.Parallelogram;

/**
 * A command that puts a specified gamepiece in its node.
 */
public class PutGamepiece extends CommandBase{

    private double height;
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
        gamePieceChooser = new SendableChooser<>();
        innitChooser();
        
    }
    /**
     * Initializes gamepiece chooser.
     */
    private void innitChooser() {
        gamePieceChooser.setDefaultOption("Cone", GamePiece.CONE);
        gamePieceChooser.addOption("Cube", GamePiece.CUBE);

    }

    @Override
    public void initialize() {

        gamePiece = gamePieceChooser.getSelected();

        if (gamePiece.getValue()==1) {
            this.height = Constants.CONE_HEIGHT;
        }
        else {
            this.height = Constants.CUBE_HEIGHT;
        }

        goToHeight = new GoToHeight(parallelogram, height, true);
        goToHeight.schedule();
    }
    
}

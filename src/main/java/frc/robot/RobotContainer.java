// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.chassis.Drive;
import frc.robot.commands.chassis.GotoCommunity;
import frc.robot.commands.chassis.GotoLoadingZone;
import frc.robot.commands.chassis.GotoNodes;
import frc.robot.commands.chassis.LeaveCommunity;
import frc.robot.commands.chassis.RampTest;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;
import frc.robot.utils.GamePiece;
import frc.robot.utils.UtilsGeneral;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static RobotContainer instance;
    public static final CommandXboxController main = new CommandXboxController(0);
    private final CommandXboxController secondary = new CommandXboxController(1);
    private final Chassis chassis;
    private final Parallelogram parallelogram;
    private final Gripper gripper;
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private Color LedLastColor;
    private GenerateAutonomous generateAutonomous;
    private GotoNodes gotoNodes;
    private LeaveCommunity leaveCommunity;

    private GamePiece gamePiece = GamePiece.CUBE;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        chassis = new Chassis();
        parallelogram = new Parallelogram();
        chassis.setDefaultCommand(new Drive(chassis, main.getHID()));
        SmartDashboard.putData((Sendable) chassis.getDefaultCommand());
        SmartDashboard.putData(chassis);
        gripper = new Gripper(GripperConstants.MOTOR_ID);
        gotoNodes = new GotoNodes(chassis, secondary ,parallelogram);
        leaveCommunity = new LeaveCommunity(chassis);
        SmartDashboard.putData(gripper);
        configureButtonBindings();
        
        leds = new AddressableLED(0);
        leds.setLength(126);
        buffer = new AddressableLEDBuffer(126);
        leds.start();

        SmartDashboard.putData(CommandScheduler.getInstance());

        generateAutonomous = new GenerateAutonomous(gotoNodes, leaveCommunity, gripper,parallelogram, chassis);
    }

    /**
     * Returns the instance of the RobotContainer class.
     * 
     * @return the instance of the RobotContainer class.
     */
    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick}
     * or {@link XboxController}), and then passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        Command load = gripper.getOpenCommand().alongWith(new GotoLoadingZone(chassis, secondary,
        parallelogram.getGoToAngleCommand(Constants.LOADING_ANGLE)))
                .andThen(gripper.getCloseCommand());

        Command unload = new GotoCommunity(chassis)
                .andThen(new GotoNodes(chassis, secondary, parallelogram)
                        .andThen(gripper.getOpenCommand()));

        load = load.until(() -> UtilsGeneral.hasInput(main.getHID()))
                .andThen((new InstantCommand(()->parallelogram.getGoBackCommand().schedule())));
        unload = unload.until(() -> UtilsGeneral.hasInput(main.getHID()))
                .andThen(new InstantCommand(()->parallelogram.getGoBackCommand().schedule()));

        main.leftBumper().onTrue(new InstantCommand(()-> gripper.getCloseCommand().schedule()));
        main.rightBumper().onTrue(new InstantCommand(()->gripper.getOpenCommand().schedule()));

        load.setName("Load");

        unload.setName("Unload");
    

        main.a().onTrue(load);
        main.x().onTrue(unload);
        main.y().onTrue(new InstantCommand(()-> parallelogram.getGoBackCommand().schedule()));
        main.povRight().onTrue(parallelogram.getGoToAngleCommand(Constants.DEPLOY_ANGLE1));
        main.povUp().onTrue(parallelogram.getGoToAngleCommand(Constants.LOADING_ANGLE));
        main.povDown().onTrue(new StartEndCommand(chassis::setRampPosition, chassis::stop, chassis).until(() -> UtilsGeneral.hasInput(main.getHID())));
        main.povLeft().onTrue(parallelogram.getGoToAngleCommand(Constants.DEPLOY_HIGH_CUBES1));

        secondary.leftBumper().onTrue(new InstantCommand(() -> {
            if (!buffer.getLED(0).equals(new Color(168, 230, 0))) {
                for (int i = 0; i < 126; i++) {
                    buffer.setRGB(i, 168, 230, 0);
                }
                gamePiece = GamePiece.CUBE;
            } else {
                for (int i = 0; i < 126; i++) {
                    buffer.setRGB(i, 0, 0, 0);
                }
            }
            leds.setData(buffer);
        }).ignoringDisable(true));

        secondary.rightBumper().onTrue(new InstantCommand(() -> {
            if (!buffer.getLED(0).equals(new Color(255, 0, 140))) {
                for (int i = 0; i < 126; i++) {
                    buffer.setRGB(i, 255, 0, 140);
                }
                gamePiece = GamePiece.CONE;
            } else {
                for (int i = 0; i < 126; i++) {
                    buffer.setRGB(i, 0, 0, 0);
                }
            }

            leds.setData(buffer);
        }).ignoringDisable(true));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    //TODO: return noraml auto command
    public Command getAutonomousCommand() {
       //return generateAutonomous.getAutonomous();
       return new RampTest(chassis).andThen(new StartEndCommand(chassis::setRampPosition, () -> {}, chassis));
    }
    public void onTeleopInit() {
        chassis.getDefaultCommand().schedule();
        parallelogram.getCalibrateCommad().schedule();
    }
}
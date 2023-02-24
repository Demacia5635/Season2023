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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.GenerateAutonomous;
import frc.robot.commands.chassis.Drive;
import frc.robot.commands.chassis.GotoCommunity;
import frc.robot.commands.chassis.GotoLoadingZone;
import frc.robot.commands.chassis.GotoNodes;
import frc.robot.commands.chassis.GotoRamp;
import frc.robot.commands.chassis.LeaveCommunity;
import frc.robot.commands.parallelogram.GoToAngle;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;
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

    private final CommandXboxController main = new CommandXboxController(0);
    private final CommandXboxController secondary = new CommandXboxController(1);
    private final Chassis chassis;
    private final Parallelogram parallelogram;
    private final Gripper gripper;
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private Color LedLastColor;
    private GenerateAutonomous generateAutonomous;
    private GotoNodes gotoNodes;

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
        gotoNodes = new GotoNodes(chassis, secondary ,() -> new GoToAngle(parallelogram, Constants.DEPLOY_ANGLE));
        SmartDashboard.putData(gripper);
        configureButtonBindings();

        leds = new AddressableLED(0);
        leds.setLength(64);
        buffer = new AddressableLEDBuffer(64);
        leds.start();

        SmartDashboard.putData(CommandScheduler.getInstance());

        generateAutonomous = new GenerateAutonomous(gotoNodes, gripper, chassis);
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
                new GoToAngle(parallelogram, Constants.LOADING_ANGLE)))
                .andThen(gripper.getCloseCommand(), new WaitCommand(0.2));

        Command unload = new GotoCommunity(chassis)
                .andThen(gotoNodes)
                        .andThen(
                                gripper.getOpenCommand());

        load = load.until(() -> UtilsGeneral.hasInput(main.getHID()))
                .andThen((new InstantCommand(()->parallelogram.getCalibrateCommad().schedule())));
        unload = unload.until(() -> UtilsGeneral.hasInput(main.getHID()))
                .andThen(new InstantCommand(()->parallelogram.getCalibrateCommad().schedule()));

        main.leftBumper().onTrue(new InstantCommand(()-> gripper.getSwitchPositionCommand().schedule()));
        main.rightBumper().onTrue(parallelogram.getCalibrateCommad());

        load.setName("Load");
        unload.setName("Unload");
    

        main.a().onTrue(load);
        main.x().onTrue(unload);
        main.b().onTrue(new GoToAngle(parallelogram, Constants.DEPLOY_ANGLE));
        main.y().onTrue(new GoToAngle(parallelogram, Constants.LOADING_ANGLE));

        main.povDown().onTrue(new InstantCommand(chassis::setRampPosition));
        main.povLeft().onTrue(new InstantCommand(chassis::setDefaultNeutral));

        secondary.rightBumper().onTrue(new InstantCommand(() -> {
            if (!buffer.getLED(0).equals(new Color(168, 0, 230))) {
                for (int i = 0; i < 64; i++) {
                    buffer.setRGB(i, 168, 0, 230);
                }
                LedLastColor = new Color(168, 0, 230);
            } else {
                for (int i = 0; i < 64; i++) {
                    buffer.setRGB(i, 255, 140, 0);
                }
                LedLastColor = new Color(255, 140, 0);
            }
            leds.setData(buffer);
        }).ignoringDisable(true));

        secondary.leftBumper().onTrue(new InstantCommand(() -> {
            if (!buffer.getLED(0).equals(new Color(0, 0, 0))) {
                for (int i = 0; i < 64; i++) {
                    buffer.setRGB(i, 0, 0, 0);
                }
            } else {
                for (int i = 0; i < 64; i++) {
                    buffer.setRGB(i, (int) (LedLastColor.red * 255), (int) (LedLastColor.green * 255),
                            (int) (LedLastColor.blue * 255));
                }
            }

            leds.setData(buffer);
        }).ignoringDisable(true));


        // controller.leftBumper().onTrue(loadIfInPlace);
        // controller.rightBumper().onTrue(unloadIfInPlace);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return generateAutonomous.getAutonomous();
    }

    public void onTeleopInit() {
        chassis.setDefaultNeutral();
        parallelogram.getCalibrateCommad().schedule();
    }

    public void onAutoInit() {
        chassis.setNeutral(true);
    }
}
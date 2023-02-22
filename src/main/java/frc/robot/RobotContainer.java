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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.chassis.Drive;
import frc.robot.commands.chassis.GotoCommunity;
import frc.robot.commands.chassis.GotoLoadingZone;
import frc.robot.commands.chassis.GotoNodes;
import frc.robot.commands.chassis.LeaveCommunity;
import frc.robot.commands.chassis.LeaveCommunity.TopOrBottom;
import frc.robot.commands.parallelogram.CalibrateParallelogram;
import frc.robot.commands.parallelogram.GoToAngle;
import frc.robot.commands.parallelogram.PickUp;
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

    private final Command autoCommand;

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
        SmartDashboard.putData(gripper);
        configureButtonBindings();

       autoCommand = new GotoNodes(chassis, secondary, new GoToAngle(parallelogram, Constants.DEPLOY_ANGLE)).andThen(gripper.getOpenCommand())
               .andThen(new CalibrateParallelogram(parallelogram)).andThen(new LeaveCommunity(chassis, TopOrBottom.BOTTOM).asProxy()).andThen(() -> {
                        System.out.println("auto ended");
                });
        
        leds = new AddressableLED(0);
        leds.setLength(64);
        buffer = new AddressableLEDBuffer(64);
        leds.start();

        SmartDashboard.putData("pickup", new PickUp(parallelogram, chassis));
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
                .andThen(gripper.getCloseCommand());

        Command unload = new GotoCommunity(chassis)
                .andThen(new GotoNodes(chassis, secondary, new GoToAngle(parallelogram, Constants.DEPLOY_ANGLE)).andThen(
                        gripper.getOpenCommand()));

        load = load.until(() -> UtilsGeneral.hasInput(main.getHID())).andThen(new CalibrateParallelogram(parallelogram));
        unload = unload.until(() -> UtilsGeneral.hasInput(main.getHID())).andThen(new CalibrateParallelogram(parallelogram));

        main.rightBumper().onTrue(gripper.getSwitchPositionCommand());
        main.leftBumper().onTrue(new CalibrateParallelogram(parallelogram));

        main.a().onTrue(load);
        main.b().onTrue(unload);
        main.x().onTrue(new GoToAngle(parallelogram, Constants.DEPLOY_ANGLE));
        main.y().onTrue(new GoToAngle(parallelogram, Constants.LOADING_ANGLE));

        secondary.rightBumper().onTrue(new InstantCommand(() -> {
            if (!buffer.getLED(0).equals(new Color(168, 0, 230))) {
                for (int i = 0; i < 64; i++) {
                    buffer.setRGB(i, 168, 0, 230);
                }
            } else {
                for (int i = 0; i < 64; i++) {
                    buffer.setRGB(i, 255, 140, 0);
                }
            }
            leds.setData(buffer);
        }).ignoringDisable(true));

        secondary.leftBumper().onTrue(new InstantCommand(() -> {
            for (int i = 0; i < 64; i++) {
                buffer.setRGB(i, 0, 0, 0);
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
        return autoCommand;
    }

    public void onTeleopInit() {
        new CalibrateParallelogram(parallelogram).schedule();
    }
}
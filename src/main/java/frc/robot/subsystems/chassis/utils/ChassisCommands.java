// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis.utils;

import java.util.Arrays;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.KeepPosition;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;

/** Add your docs here. */
public class ChassisCommands {
    
    public static Command createPathFollowingCommand(PathPlannerTrajectory trajectory,
    boolean resetPose, boolean keepPosition, Command onTrajectoryEnd) {
var command = new SequentialCommandGroup(
        new PPSwerveControllerCommand(
                trajectory,
                Chassis.GetInstance()::getPose,
                ChassisConstants.KINEMATICS,
                new PIDController(ChassisConstants.AUTO_TRANSLATION_KP, ChassisConstants.AUTO_TRANSLATION_KI,
                        0),
                new PIDController(ChassisConstants.AUTO_TRANSLATION_KP, ChassisConstants.AUTO_TRANSLATION_KI,
                        0),
                new PIDController(ChassisConstants.AUTO_ROTATION_KP, ChassisConstants.AUTO_ROTATION_KI, 0),
                (ChassisSpeeds)->{
                    try{Chassis.GetInstance().setModuleStates(ChassisSpeeds);}
                    catch(Exception e){System.out.println("Chassis instance is null");}},
                Chassis.GetInstance())
                .andThen(
                        (keepPosition ? new KeepPosition(Chassis.GetInstance(),
                                new Pose2d(trajectory.getEndState().poseMeters.getTranslation(),
                                        trajectory.getEndState().holonomicRotation))
                                : new InstantCommand())
                                .andThen(onTrajectoryEnd)),
        new InstantCommand(() -> System.out.println(("Trajectory ended"))));
    return command;
}

public static Command createPathFollowingCommand(PathPlannerTrajectory trajectory,
    boolean resetPose, boolean keepPosition) {
return createPathFollowingCommand(trajectory, resetPose, keepPosition, new InstantCommand());
}

/**
* Creates a path following command
* 
* @param path      The path to follow
* @param events    The events to run on the markers in the path
* @param resetPose Whether to reset the pose of the robot at the start of the
*                  command
* @return the path following command
*/
public static Command createPathFollowingCommand(String path, boolean resetPose) {
var trajectory = PathPlanner.loadPath(path, ChassisConstants.PATH_CONSTRAINTS);
return createPathFollowingCommand(trajectory, resetPose, true);
}

/**
* Creates a path following command
* 
* @param path         The path to follow
* @param events       The events to run on the markers in the path
* @param resetPose    Whether to reset the pose of the robot at the start of
*                     the command
* @param keepPosition Whether to keep the position of the robot at the end of
*                     the command
* @return the path following command
*/
public static Command createPathFollowingCommand(String path, boolean resetPose,
    boolean keepPosition) {
var trajectory = PathPlanner.loadPath(path, ChassisConstants.PATH_CONSTRAINTS);
return createPathFollowingCommand(trajectory, resetPose, keepPosition);
}

/**
* Creates a path following command
* 
* @param path The path to follow
* @return the path following command
*/
public static Command createPathFollowingCommand(String path) {
return createPathFollowingCommand(path, true);
}

/**
* Creates a path following command
* 
* @param points The points to follow (including the current position)
* @return the path following command
*/
public static Command createPathFollowingCommand(PathPoint... points) {
if (points.length < 2)
    return null;
var trajectory = PathPlanner.generatePath(ChassisConstants.PATH_CONSTRAINTS, Arrays.asList(points));
return createPathFollowingCommand(trajectory, false, true);
}

/**
* Creates a path following command
* 
* @param points The points to follow (including the current position)
* @return the path following command
*/
public static Command createPathFollowingCommand(PathConstraints constraints, PathPoint... points) {
if (points.length < 2)
    return null;
var trajectory = PathPlanner.generatePath(constraints, Arrays.asList(points));
return createPathFollowingCommand(trajectory, false, true);
}

/**
* Creates a path following command
* 
* @param keepPosition Whether to keep the position of the robot at the end of
*                     the command
* @param points       The points to follow (including the current position)
* @return the path following command
*/
public static Command createPathFollowingCommand(boolean keepPosition, PathPoint... points) {
if (points.length < 2)
    return null;
var trajectory = PathPlanner.generatePath(ChassisConstants.PATH_CONSTRAINTS, Arrays.asList(points));
return createPathFollowingCommand(trajectory, false, keepPosition);
}

public static Command createPathFollowingCommand(boolean keepPosition, PathConstraints constraints, PathPoint... points) {
if (points.length < 2)
    return null;
var trajectory = PathPlanner.generatePath(constraints, Arrays.asList(points));
return createPathFollowingCommand(trajectory, false, keepPosition);
}

public static Command createPathFollowingCommand(Command onTrajectoryEnd, PathPoint... points) {
if (points.length < 2)
    return null;
var trajectory = PathPlanner.generatePath(ChassisConstants.PATH_CONSTRAINTS, Arrays.asList(points));
return createPathFollowingCommand(trajectory, false, true, onTrajectoryEnd);
}

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
      swerveSubsystem, 
      () -> -driverJoystick.getRawAxis(OIConstants.DRIVER_Y_AXIS), 
      () -> driverJoystick.getRawAxis(OIConstants.DRIVER_X_AXIS),
      () -> driverJoystick.getRawAxis(OIConstants.DRIVER_ROTATION_AXIS),
      () -> !driverJoystick.getRawButton(OIConstants.DRIVER_FIELD_ORIENTED_TOGGLE_BUTTON))); //opposite value to set field-oriented as default

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading()); //reset the direction of the field's reference frame
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    //1. create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND, 
      AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(DriveConstants.DRIVE_KINEMATICS);
    
    //2. generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory( //update this after you learn pathWeaver
      //initial coordinates
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        //the robot will travel to these points in this order
        new Translation2d(1, 0),
        new Translation2d(1, -1)),
      //do a 180 after reaching this last point
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig);

    //3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kP_X_CONTROLLER, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kP_Y_CONTROLLER, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kP_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //4. Constuct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      swerveSubsystem::getPose,
      DriveConstants.DRIVE_KINEMATICS,
      xController,
      yController,
      thetaController,
      swerveSubsystem::setModuleStates,
      swerveSubsystem);

    //5. add some init and closing, and return everything
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}

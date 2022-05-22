// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ModuleConstants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 8.14;
        public static final double TURNING_MOTOR_GEAR_RATIO = 1 / 12.8;
        public static final double DRIVE_MOTOR_ROTATIONS_TO_METERS = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double TURNING_MOTOR_ROTATIONS_TO_RADIANS = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
        public static final double DRIVE_MOTOR_RPM_TO_METERS_PER_SECOND = DRIVE_MOTOR_ROTATIONS_TO_METERS / 60;
        public static final double TURNING_MOTOR_RPM_TO_RADIANS_PER_SECOND = TURNING_MOTOR_ROTATIONS_TO_RADIANS / 60;
        public static final double kP_TURNING = 0.5; //arbitrary value, will need to actually tune on an actual robot
    }
    
    public static final class DriveConstants {
        public static final double TRACK_WIDTH = Units.inchesToMeters(21); //arbitrary value, will need to change once drivetrain is built
        //distance between right and left wheels
        public static final double WHEEL_BASE = Units.inchesToMeters(25.5); //arbitrary value, will need to change once drivetrain is built
        //distance between front and back wheels
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2));

        public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 0; //arbitrary port, will need to change once configured via phoenix tuner
        public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 1; //arbitrary port, will need to change once configured via phoenix tuner
        public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 2; //arbitrary port, will need to change once configured via phoenix tuner
        public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 3; //arbitrary port, will need to change once configured via phoenix tuner

        public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 4; //arbitrary port, will need to change once configured via phoenix tuner
        public static final int BACK_LEFT_TURNING_MOTOR_PORT = 5; //arbitrary port, will need to change once configured via phoenix tuner
        public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 6; //arbitrary port, will need to change once configured via phoenix tuner
        public static final int BACK_RIGHT_TURNING_MOTOR_PORT = 7; //arbitrary port, will need to change once configured via phoenix tuner
        
        public static final boolean FRONT_LEFT_TURNING_MOTOR_REVERSED = true;
        public static final boolean BACK_LEFT_TURNING_MOTOR_REVERSED = true;
        public static final boolean FRONT_RIGHT_TURNING_MOTOR_REVERSED = true;
        public static final boolean BACK_RIGHT_TURNING_MOTOR_REVERSED = true;

        public static final boolean FRONT_LEFT_DRIVE_MOTOR_REVERSED = true;
        public static final boolean BACK_LEFT_DRIVE_MOTOR_REVERSED = true;
        public static final boolean FRONT_RIGHT_DRIVE_MOTOR_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_MOTOR_REVERSED = false;
        
        public static final int FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 8; //arbitrary port, will need to change once configured via phoenix tuner
        public static final int BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 9; //arbitrary port, will need to change once configured via phoenix tuner
        public static final int FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 10; //arbitrary port, will need to change once configured via phoenix tuner
        public static final int BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 11; //arbitrary port, will need to change once configured via phoenix tuner
        
        public static final boolean FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;

        public static final double FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -0.254;
        public static final double BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -1.252;
        public static final double FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -1.816;
        public static final double BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -4.811;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 5;
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;

        public static final double TELEOP_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4; //robot can only move at 25% of its max potential speed
        public static final double TELEOP_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 
            PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4; //robot can only turn at 25% of its max potential turn speed
        public static final double TELEOP_MAX_ACCELERATION_UNITS_PER_SECOND = 3;
        public static final double TELEOP_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;
    }

    public static final class AutoConstants {
        public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;
        public static final double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 
            DriveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 10;
        public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double AUTO_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI / 4;
        public static final double kP_X_CONTROLLER = 1.5; //arbitrary value, re-tune with actual robot
        public static final double kP_Y_CONTROLLER = 1.5; //arbitrary value, re-tune with actual robot
        public static final double kP_THETA_CONTROLLER = 3;

        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                AUTO_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;

        public static final int DRIVER_Y_AXIS = 1;
        public static final int DRIVER_X_AXIS = 0;
        public static final int DRIVER_ROTATION_AXIS = 4;
        public static final int DRIVER_FIELD_ORIENTED_TOGGLE_BUTTON = 1;
        
        public static final double DEADBAND = 0.05;
    }
}

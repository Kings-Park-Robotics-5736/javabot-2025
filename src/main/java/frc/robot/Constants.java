// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.Limits;
import frc.robot.utils.Types.PidConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.
 * This class should not be used for any other purpose. All constants
 * should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kDt = 0.02;

  public static final class DriveConstants {

    public static final int kFrontLeftDriveMotorPort = 2;
    public static final int kRearLeftDriveMotorPort = 4;
    public static final int kFrontRightDriveMotorPort = 0;
    public static final int kRearRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 3;
    public static final int kRearLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 1;
    public static final int kRearRightTurningMotorPort = 7;

    public static final int kFrontLeftTurningEncoderPort = 1;
    public static final int kRearLeftTurningEncoderPort = 2;
    public static final int kFrontRightTurningEncoderPort = 0;
    public static final int kRearRightTurningEncoderPort = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveReversed = true;
    public static final boolean kRearLeftDriveReversed = false;
    public static final boolean kFrontRightDriveReversed = true;
    public static final boolean kRearRightDriveReversed = true;

    public static final double kFrontLeftAngleOffset = -0.189941;// 0.358643; //unit is from -1 to 1, normalized
    public static final double kFrontRightAngleOffset = -0.920654;
    public static final double kBackLeftAngleOffset = -0.733887;
    public static final double kBackRightAngleOffset = 0.487793;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.60325;

    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.55245;

    

    public static final String kCanName = "Canivore";

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    public static final double ksVoltsTurning = .10059;
    public static final double kvVoltSecondsPerMeterTurning = 0.3749;
    public static final double kaVoltSecondsSquaredPerMeterTurning = 0.1142;
    public static final double ksVoltsDrive = 0.0532576;
    public static final double kvVoltsDrive = 2.5721;
    public static final double kaVoltsDrive = 0.455225;
    public static final double kPDrive = 1.4120225;

    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxRotationSpeedMetersPerSecond = 6;

    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
  }



  public static final class ClimbConstants {
    // TODO tune these values
    public static final int kForwardSpeed = 4000;
    public static final int kReverseSpeed = -3000;
  }


  public static final class ElevatorConstants{
    public static final byte kLeaderDeviceId = 9;
    public static final byte kFollowerDeviceId = 10;

    public static final PidConstants kPidValues = new PidConstants(0.5, 0, 0.007);
    public static final Limits kLimits = new Limits(0, 60);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.06, 1.2666, 0, .19);

    public static final int kMaxVelocity = 100;
    public static final int kMaxAcceleration = 100;

    public static final double kStaleTolerance = .5;
    public static final double kDiffThreshold = 0.15;
    public static final int kStaleThreshold = 10;
    public static final double kPositionTolerance = 1.0;


    public static final double kL1Position = 0.0;
    public static final double kL2Position = 59.5;
    public static final double kL3Position = 59.5;
    public static final double kL4Position = 59.5;
    public static final double kIntakePosition = 59.5;
  }

  public static final class ArmConstants {

    public static final byte kMotorID = 14;

    public static final String kCanName = "rio";

    public static final double kMaxVelocity = (double) 2 *  Math.PI; //max velocity is 90 deg / sec
    public static final double kMaxAcceleration = (double) 4* Math.PI; 
    public static final double kMaxJerk = (double) 20 * Math.PI;
    public static final PidConstants kPidValues = new PidConstants(15, 0, 0);
    
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.06, 1.2666, 0, .19);

   
    public static final double kPositionTolerance = Math.toRadians(1.0);
    public static final double kStaleTolerance = Math.toRadians(3);
    public static final double kDiffThreshold = Math.toRadians(.9);
    public static final int kStaleThreshold = 20;


    public static final Limits kLimits = new Limits(Math.toRadians(-100),Math.toRadians(180));

    public static final double L1Angle = Math.toRadians(0);
    public static final double L2Angle = Math.toRadians(25);
    public static final double L3Angle = Math.toRadians(25);
    public static final double L4Angle = Math.toRadians(120);
    public static final double intakeAngle = Math.toRadians(0);


  }



  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 15 * 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 15 * 2 * Math.PI;

    public static final int kEncoderResolution = 2048;
    public static final double kWheelRadius = 0.0508;
    public static final double kDriveGearRatio = 6.75;

    public static final double kPModuleTurningController = 7.9245;
    public static final double kPModuleTurningNoController = 5;

  }

  public static final class IOConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kActionControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 5;
    public static final double kPYController = 5;
    public static final double kPThetaController = 5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
        new PIDConstants(5.0, 0, 0), // Translation constants
        new PIDConstants(5.0, 0, 0) // Rotation constants
    );
  }

  public static final class CenterToFieldPositionConstants {
    public static final double kMaxSpeedMetersPerSecond = 9;
    public static final double kMaxAccelerationMetersPerSecondSquared = 9;
    public static final PidConstants kPidValues = new PidConstants(.75, 0, 0.00);
  }

  public static final class DriveToTargetCommandConstants {

    // after 30 times of not seeing target after seeing it, declare it intaked
    public static final int kGotTargetThreshold = 50;

    public static final double kMaxRotation = Math.PI / 4; // 45 degrees
    public static final PidConstants kPidValues = new PidConstants(1, 0, 0.00);

  }

  public static final class CustomDriveDistanceCommandConstants {
    public static final PidConstants kPidValues = new PidConstants(.5, .1, 0.00);

  }

  public static final class LEDConstants {
    public static final int CANdleID = 20;
    public static final int PWMLedId = 0;
    public static final int LED_Count = 150;
    public static final int[] redRGB = {255,0,0};
    public static final int[] GreenRGB = {0,255,0};
     public static final int[] BlueRGB = {0,0,255};
  }
}

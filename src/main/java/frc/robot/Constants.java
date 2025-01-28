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

  public static final class IntakeConstants {

    public static final int kForwardSpeed = 4000;
    public static final int kReverseSpeed = -3000;
  }

  public static final class ClimbConstants {
    // TODO tune these values
    public static final int kForwardSpeed = 4000;
    public static final int kReverseSpeed = -3000;
  }

  public static final class LeftClimbConstants {

    public static final byte kDeviceId = 2;
    // TODO tune these values
    public static final PidConstants kPidValues = new PidConstants(0.00001, 0, 0);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.06368, 0.12005, 0.0034381);
  }

  public static final class RightClimbConstants {

    public static final byte kDeviceId = 17;
    // TODO tune these values
    public static final PidConstants kPidValues = new PidConstants(3.7415E-05, 0, 0);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.19204, 0.12201, 0.0082481);
  }

  public static final class LowerIntakeConstants {

    public static final byte kDeviceId = 1;

    public static final PidConstants kPidValues = new PidConstants(5.1248E-06, 0, 0);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.2672, 0.12568, 0.0065158);
  }

  public static final class UpperIntakeConstants {

    public static final byte kDeviceId = 3;

    public static final PidConstants kPidValues = new PidConstants(5.1248E-06, 0, 0);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.2672, 0.12568, 0.0065158);
  }

  public static final class ArmConstants {

    public static final byte kLeaderDeviceId = 14;
    public static final byte kFollowerDeviceId = 10;

    public static final byte kNoteSensorDIO = 1;
    public static final String kCanName = "rio";

    public static final double kMaxVelocity = (double) 2 *  Math.PI; //max velocity is 90 deg / sec
    public static final double kMaxAcceleration = (double) 2.5*  Math.PI; //max accel is 45 deg/sec^2
    public static final PidConstants kPidValues = new PidConstants(15/*5.3718*/, 0, 0);
    //public static final PidConstants kPidValues = new PidConstants(30/*5.3718*/, 80, .1);
    //public static final FeedForwardConstants kFFValues = new FeedForwardConstants(1.2, .52879, 1.0435, 3.991);
    //public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.42502, 2.8721, 0.92093, -1.5);
    //public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.87319, 1.8992, 0.55601, -3.153);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.06, 1.2666, 0, .19);

    //kg 1.56
    //kv .62
    //ka .08

    //public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0, .62, 0.08, 1.56);



    //public static final FeedForwardConstants kFFValues = new FeedForwardConstants(.20035, 3.2988, 0.52066, 0.48143);
   
    public static final double kPositionTolerance = Math.toRadians(1.0);
    public static final double kStaleTolerance = Math.toRadians(3);
    public static final double kDiffThreshold = Math.toRadians(.9);
    public static final int kStaleThreshold = 20;

    public static final double falconOffsetAngleDegrees = 35;
    public static final double armEncoderOffsetAngleDegrees = 35;//-203;

    public static final Limits kLimits = new Limits(Math.toRadians(180), Math.toRadians(-120));

    public static final double falconErrorThresh = Math.toRadians(.5);
    public static final double falconErrorCount = 5;

    public static final double intakeAngle = Math.toRadians(35);
    public static final double ampAngle = Math.toRadians(-104);
    public static final double scorpionAngle = Math.toRadians(-111);
    public static final double TrapAngle = Math.toRadians(-115);
    public static final double FixedAngle = Math.toRadians(0);


  }

  public static final class ShooterConstants {

    public static final int kDesiredSpeed = 4000; //RPM
    public static final int kAmpSpeed = 500;
    public static final int scorpionSpeed = 1500;
    public static final int kReverseSpeed = -1500; //RPM
    public static final int kIdleSpeed = 2000; //RPM
    public static final int kTolerance = 150; //RPM
  }

  public static final class LeftShooterConstants {
    public static final byte kDeviceId = 1;

    public static final PidConstants kPidValues = new PidConstants(0.046382, 0, 0);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.17665, 0.11767, 0.0067599);
  }

  public static final class RightShooterConstants {
    public static final byte kDeviceId = 0;

    public static final PidConstants kPidValues = new PidConstants(0.046382, 0, 0);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.17665, 0.11767, 0.0067599);
  }

  public static final class kickupConstants {

    public static final int kForwardSpeed = -2000;
    public static final int kReverseSpeed = 1000;
  }

  public static final class RightKickupConstants {
    public static final byte kDeviceId = 2;

    public static final PidConstants kPidValues = new PidConstants(0.001873, 0, 0);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.058711, 0.11659, 0.0016706);
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

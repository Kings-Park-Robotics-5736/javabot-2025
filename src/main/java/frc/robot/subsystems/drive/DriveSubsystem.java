// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.vision.Limelight;

public class DriveSubsystem extends SubsystemBase {

  /****************************************************
   * Define the 4 swerve modules
   ****************************************************/
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kCanName,
      DriveConstants.kFrontLeftTurningEncoderReversed,
      DriveConstants.kFrontLeftDriveReversed,
      DriveConstants.kFrontLeftAngleOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kCanName,
      DriveConstants.kRearLeftTurningEncoderReversed,
      DriveConstants.kRearLeftDriveReversed,
      DriveConstants.kBackLeftAngleOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kCanName,
      DriveConstants.kFrontRightTurningEncoderReversed,
      DriveConstants.kFrontRightDriveReversed,
      DriveConstants.kFrontRightAngleOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kCanName,
      DriveConstants.kRearRightTurningEncoderReversed,
      DriveConstants.kRearRightDriveReversed,
      DriveConstants.kBackRightAngleOffset);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(0, "Canivore");

  /**
   * @brief This is a flag to lockout the joystick control of the robot.
   *        This is used when the robot is running an auto-command.
   *        It is needed, or else the joystick will fight the auto-command with
   *        its default 0's.
   */
  private boolean m_joystickLockoutTranslate;
  private boolean m_joystickLockoutRotate;
  private double m_rotateLockoutValue;
  private boolean m_joystickLockoutRotateFieldOriented;
  private double m_transXLockoutValue;
  private double m_transYLockoutValue;
  private long m_lastPoseUpdate;

  private boolean pathPlannerInitSuccess;
  private boolean visionEnabled;

  private Limelight m_limelight;
  private Limelight m_limelight_side;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Voltage volts) -> {
            m_frontLeft.getDriveMotor().setVoltage(volts.in(Volts));
            m_frontRight.getDriveMotor().setVoltage(volts.in(Volts));
            m_rearLeft.getDriveMotor().setVoltage(volts.in(Volts));
            m_rearRight.getDriveMotor().setVoltage(volts.in(Volts));
            m_frontLeft.lockTurningAtZero();
            m_frontRight.lockTurningAtZero();
            m_rearLeft.lockTurningAtZero();
            m_rearRight.lockTurningAtZero();
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            log.motor("drive-front-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        m_frontLeft.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(m_frontLeft.getPosition().distanceMeters, Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_frontLeft.getState().speedMetersPerSecond, MetersPerSecond));
            log.motor("drive-front-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        m_frontRight.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(m_frontRight.getPosition().distanceMeters, Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_frontRight.getState().speedMetersPerSecond, MetersPerSecond));
            log.motor("drive-rear-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        m_rearLeft.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(m_rearLeft.getPosition().distanceMeters, Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_rearLeft.getState().speedMetersPerSecond, MetersPerSecond));
            log.motor("drive-rear-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        m_rearRight.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(m_rearRight.getPosition().distanceMeters, Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_rearRight.getState().speedMetersPerSecond, MetersPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("drive")
          this));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Limelight ll, Limelight l2) {

    m_joystickLockoutTranslate = false;
    m_joystickLockoutRotate = false;
    m_joystickLockoutRotateFieldOriented = false;

    visionEnabled=true;
    SmartDashboard.putBoolean("LimelightVisionEnabled", visionEnabled);

    m_transXLockoutValue = 0;
    m_transYLockoutValue = 0;

    m_limelight = ll;
    m_limelight_side = l2;

    m_lastPoseUpdate = 0;

    m_gyro.setYaw(180);

    RobotConfig config;
    pathPlannerInitSuccess = true;
    try{
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds, 
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        Constants.AutoConstants.pathFollowerConfig,
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      pathPlannerInitSuccess = false;
    }

   

    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        new Rotation2d(Units.degreesToRadians(180)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(180))),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  }

  /**
   * Update odometry from the current positions and angles.
   */
  public void updateOdometry() {

    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    if(visionEnabled){
      addLimelightVisionMeasurement(m_limelight, true);
      if (m_limelight_side != null) {
        addLimelightVisionMeasurement(m_limelight_side, false);
      }
    }

    SmartDashboard.putNumber("Gyro Rotation", m_gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Pose Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Robot Pose X", getPose().getX());
    SmartDashboard.putNumber("Robot Pose Y", getPose().getY());

  }


  private void addLimelightVisionMeasurement(Limelight ll, boolean primary) {

    double[] pose;
    double transStd;
    double rotStd;
    long currentUpdateTime = 0;
    TimestampedDoubleArray poseWithTime;

    if (!ll.getIsPipelineAprilTag()) {
      return;
    }
    var alliance = DriverStation.getAlliance();
    boolean validPose = ll.checkValidTarget();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      poseWithTime = ll.getBotPoseBlue();
    } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      poseWithTime = ll.getBotPoseBlue();
    } else {
      validPose = false;
      pose = new double[6]; // empty 6 position array
      poseWithTime = new TimestampedDoubleArray(0, 0, pose);
    }

    if (!validPose) {
      return;
    }

    currentUpdateTime = poseWithTime.timestamp;

    if (currentUpdateTime == m_lastPoseUpdate) {
      return;
    }
    m_lastPoseUpdate = currentUpdateTime;

    // observed bad tracking when tag is very close to edge
    double offsetX = ll.getTargetOffsetX();

    if (Math.abs(offsetX) < 32.5) {

      pose = poseWithTime.value;
      Pose2d limelightPose = ll.AsPose2d(pose);

      double[] cameraToAprilTagPose = m_limelight.getTargetPoseCameraSpace();

      if(cameraToAprilTagPose.length > 0){
        double distanceToAprilTagSquared = cameraToAprilTagPose[0] * cameraToAprilTagPose[0]
            + cameraToAprilTagPose[2] * cameraToAprilTagPose[2];
        double poseDelta = m_poseEstimator.getEstimatedPosition().getTranslation()
            .getDistance(limelightPose.getTranslation());


        if(primary){
          if (distanceToAprilTagSquared < 9 && poseDelta < .5) {
            transStd = 0.5;
            rotStd = 10;
          } else if (distanceToAprilTagSquared < 25) {
            transStd = 1.0;
            rotStd = 15;
          } else {
            transStd = 1.5;
            rotStd = 20;
          }
        }else{
            if (distanceToAprilTagSquared < 9 && poseDelta < .5) {
            transStd = 1.5;
            rotStd = 20;
          } else if (distanceToAprilTagSquared < 25) {
            transStd = 2.0;
            rotStd = 25;
          } else {
            transStd = 2.5;
            rotStd = 30;
          }
        }
        if (limelightPose.getX() > .5) {
          // Apply vision measurements. pose[6] holds the latency/frame delay
          m_poseEstimator.addVisionMeasurement(
              limelightPose,
              Timer.getFPGATimestamp() - (pose[6] / 1000.0),
              VecBuilder.fill(transStd, transStd, Units.degreesToRadians(rotStd)));
        }
      }else{
        System.out.println("Unable to add limelight measurement");
      }
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();
    SmartDashboard.getBoolean("LimelightVisionEnabled", visionEnabled);
  }


  public boolean getPathPlannerInitSuccess(){
    return pathPlannerInitSuccess;
  }
  
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // return m_odometry.getPoseMeters();
    return m_poseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    System.out.println("-----------------Odometry Reset__________________");
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    drive(xSpeed, ySpeed, rot, fieldRelative, false);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean joystick) {

    if ((joystick && !m_joystickLockoutTranslate) || !joystick) {

      if (m_joystickLockoutRotate && joystick) {
        rot = m_rotateLockoutValue;
        m_transXLockoutValue = xSpeed;
        m_transYLockoutValue = ySpeed;
        fieldRelative = m_joystickLockoutRotateFieldOriented;
      } else if (m_joystickLockoutRotate && !joystick && !m_joystickLockoutTranslate) {
        xSpeed = m_transXLockoutValue;
        ySpeed = m_transYLockoutValue;
      }
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose().getRotation())
              : new ChassisSpeeds(xSpeed, ySpeed, rot));
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    }
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, false);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void forceStop() {
    m_frontLeft.forceStop();
    m_frontRight.forceStop();
    m_rearLeft.forceStop();
    m_rearRight.forceStop();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    var currentPose = getPose();
    Pose2d newPose = new Pose2d(currentPose.getTranslation(), m_gyro.getRotation2d());
    this.resetOdometry(newPose);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingDegrees() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getHeadingInRadians() {
    return m_gyro.getRotation2d().getRadians();
  }



  public void setJoystickRotateLockout(boolean val) {
    setJoystickRotateLockout(val, false);
  }

  public void setJoystickRotateLockout(boolean val, boolean shouldBeFieldOriented) {
    m_joystickLockoutRotate = val;
    m_joystickLockoutRotateFieldOriented = shouldBeFieldOriented;
  }

  public void setJoystickTranslateLockout(boolean val) {
    m_joystickLockoutTranslate = val;
  }

  public void setRotateLockoutValue(double val) {
    m_rotateLockoutValue = val;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

}

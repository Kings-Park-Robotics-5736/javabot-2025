// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.JoystickCommandsFactory;
import frc.robot.commands.RobotCommandsFactory;
import frc.robot.commands.TrajectoryCommandsFactory;
import frc.robot.commands.drive.CenterToGoalCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.subsystems.ElevateAssembly.ArmSubsystemFalcon;
import frc.robot.subsystems.ElevateAssembly.ElevateSubsystem;
import frc.robot.subsystems.ElevateAssembly.ElevatorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.Types.GoalType;
import frc.robot.utils.Types.PositionType;
import frc.robot.utils.Types.SysidMechanism;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LEDMode;
import frc.robot.vision.PiCamera;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        // The robot's subsystems

        private final SysidMechanism enabledSysid = SysidMechanism.NONE;

        private final PiCamera m_picam = new PiCamera();
        public Limelight m_limelight = new Limelight("limelight");
        public Limelight m_limelight_side = new Limelight("limelight-side");
        public ArmSubsystemFalcon m_arm = new ArmSubsystemFalcon();
        //public EndeffectorSubsystem m_endeffector = new EndeffectorSubsystem();
        public ElevateSubsystem m_elevate = new ElevateSubsystem();
        public ElevatorSubsystem m_elevator = new ElevatorSubsystem("elevtor");

        private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_limelight, m_limelight_side);// use only 1 limelight for
                                                                                          // driving now since we dont
                                                                                          // have great measurements
                                                                                          // m_limelight_side);

       
        private final PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);
        private final SendableChooser<Command> autoChooser;

        private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
        private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
        private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

        XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);
        XboxController m_actionController = new XboxController(IOConstants.kActionControllerPort);

        private void driveWithJoystick(Boolean fieldRelative) {
                // Get the x speed. We are inverting this because Xbox controllers return
                // negative values when we push forward.

                var leftY = m_driverController.getLeftY();
                var leftX = m_driverController.getLeftX();
                var rightX = m_driverController.getRightX();
                //System.out.println("LeftY Pre = " + leftY);

                if(m_driverController.getLeftStickButton()){
                       leftY /=2; 
                       leftX /=2;
                       System.out.println("Speed Limiting");
                }

                if(m_driverController.getRightStickButton()){
                        rightX /=2;
                }
                //System.out.println("LeftY After = " + leftY);
                var xSpeed = -m_xspeedLimiter
                                .calculate(MathUtil.applyDeadband(leftY, 0.08))
                                * DriveConstants.kMaxSpeedMetersPerSecond;

                // Get the y speed or sideways/strafe speed. We are inverting this because
                // we want a positive value when we pull to the left. Xbox controllers
                // return positive values when you pull to the right by default.
                var ySpeed = -m_yspeedLimiter
                                .calculate(MathUtil.applyDeadband(leftX, 0.08))
                                * DriveConstants.kMaxSpeedMetersPerSecond;

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                        xSpeed = -xSpeed;
                        ySpeed = -ySpeed;
                }

                // Get the rate of angular rotation. We are inverting this because we want a
                // positive value when we pull to the left (remember, CCW is positive in
                // mathematics). Xbox controllers return positive values when you pull to
                // the right by default.
                final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(rightX, 0.1))
                                * DriveConstants.kMaxRotationSpeedMetersPerSecond;

                if (rot != 0){
                        SmartDashboard.putBoolean("Square to Target?", false);
                }
                if(m_driverController.getLeftBumperButton()){
                        fieldRelative = false;
                          xSpeed = -xSpeed;
                        ySpeed = -ySpeed;
                }
                m_robotDrive.drive(xSpeed, ySpeed, rot, fieldRelative, true);
        }

        private void InitializeNamedCommands() {
                NamedCommands.registerCommand("Forward1", new DriveDistanceCommand(m_robotDrive, 1));
                NamedCommands.registerCommand("Forward0.5", new DriveDistanceCommand(m_robotDrive, 0.5));
                NamedCommands.registerCommand("GrabTarget", new DriveToTargetCommand(m_robotDrive, m_picam, 2.25, 1));
                NamedCommands.registerCommand("ForceStop", Commands.runOnce(() -> m_robotDrive.forceStop()));
               
                NamedCommands.registerCommand("DriveToNote", new DriveToTargetCommand(m_robotDrive, m_picam, 1, 1));
                
                NamedCommands.registerCommand("CenterToTarget", new CenterToGoalCommand(m_robotDrive, false));
                NamedCommands.registerCommand("CenterToTargetInverse", new CenterToGoalCommand(m_robotDrive, false, true));

                NamedCommands.registerCommand("CenterToTargetInfinite", new CenterToGoalCommand(m_robotDrive, true));
                NamedCommands.registerCommand("CenterToTargetInfiniteInverse", new CenterToGoalCommand(m_robotDrive, true, true, GoalType.SPEAKER, Math.toRadians(10)));

          }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                InitializeNamedCommands(); // must do this first

                // Configure the button bindings
                switch (enabledSysid) {

                        case INTAKE_TOP:
                        case INTAKE_BOTTOM:
                               // configureButtonBindingsIntakeSysID();
                                break;
                        case DRIVE:
                                configureButtonBindingsDriveSysID();
                                break;
                        case SHOOTER_LEFT:
                                //configureButtonBindingsShooterLeftSysID();
                                break;
                        case SHOOTER_RIGHT:
                                //configureButtonBindingsShooterRightSysID();
                                break;
                        case KICKUP_RIGHT:
                                //configButtonBindingsKickupRightSysID();
                                break;
                        case ARM:
                                configButtonBindingsArmSysID();
                                break;
                        case NONE:
                        default:
                                configureButtonBindings();
                                // Configure default commands
                                PDH.setSwitchableChannel(true);
                                // Enable LEDS
                                // will this happen when we turn on/push code?
                                // or on enable
                                m_robotDrive.setDefaultCommand(
                                                // The left stick controls translation of the robot.
                                                // Turning is controlled by the X axis of the right stick.
                                                new RunCommand(
                                                                () -> driveWithJoystick(true),
                                                                m_robotDrive));
                                break;

                }

                // Set limelight LED to follow pipeline on startup
                m_limelight.setLEDMode(LEDMode.PIPELINE);

                if(m_robotDrive.getPathPlannerInitSuccess()){
                        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
                        SmartDashboard.putData("Auto Mode", autoChooser);
                }else {
                        autoChooser = null;
                }

        
        
        
        }

        public void resetArm(){
        }
        private void configureButtonBindingsDriveSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }
        /*
        private void configureButtonBindingsIntakeSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward,
                                                PositionType.TOP)
                                                .alongWith(m_intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward,
                                                                PositionType.BOTTOM)));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse,
                                                PositionType.TOP)
                                                .alongWith(m_intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse,
                                                                PositionType.BOTTOM)));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_intake.sysIdDynamic(SysIdRoutine.Direction.kForward, PositionType.TOP)
                                                .alongWith(m_intake.sysIdDynamic(SysIdRoutine.Direction.kForward,
                                                                PositionType.BOTTOM)));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_intake.sysIdDynamic(SysIdRoutine.Direction.kReverse, PositionType.TOP)
                                                .alongWith(m_intake.sysIdDynamic(SysIdRoutine.Direction.kReverse,
                                                                PositionType.BOTTOM)));

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .toggleOnTrue(m_intake.RunIntakeForwardCommand());

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .toggleOnTrue(m_intake.RunIntakeBackwardCommand());
        }

        private void configureButtonBindingsShooterLeftSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_Launcher.sysIdShooterQuasistatic(SysIdRoutine.Direction.kForward,
                                                PositionType.LEFT));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_Launcher.sysIdShooterQuasistatic(SysIdRoutine.Direction.kReverse,
                                                PositionType.LEFT));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_Launcher.sysIdShooterDynamic(SysIdRoutine.Direction.kForward,
                                                PositionType.LEFT));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_Launcher.sysIdShooterDynamic(SysIdRoutine.Direction.kReverse,
                                                PositionType.LEFT));

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .toggleOnTrue(m_Launcher.RunShooterForwardCommand());

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .toggleOnTrue(m_Launcher.RunShooterBackwardCommand());
        }

        private void configureButtonBindingsShooterRightSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_Launcher.sysIdShooterQuasistatic(SysIdRoutine.Direction.kForward,
                                                PositionType.RIGHT));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_Launcher.sysIdShooterQuasistatic(SysIdRoutine.Direction.kReverse,
                                                PositionType.RIGHT));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_Launcher.sysIdShooterDynamic(SysIdRoutine.Direction.kForward,
                                                PositionType.RIGHT));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_Launcher.sysIdShooterDynamic(SysIdRoutine.Direction.kReverse,
                                                PositionType.RIGHT));

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .toggleOnTrue(m_Launcher.RunShooterForwardCommand());

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .toggleOnTrue(m_Launcher.RunShooterBackwardCommand());
        }

        private void configButtonBindingsKickupRightSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_Launcher.sysIdKickupQuasistatic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_Launcher.sysIdKickupQuasistatic(SysIdRoutine.Direction.kReverse));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_Launcher.sysIdKickupDynamic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_Launcher.sysIdKickupDynamic(SysIdRoutine.Direction.kReverse));


                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .toggleOnTrue(m_Launcher.RunKickupForwardCommand());

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .toggleOnTrue(m_Launcher.RunKickupBackwardCommand());
        }
*/
        private void configButtonBindingsArmSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(m_arm.RunArmToPositionCommand(Math.toRadians(120)));

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .whileTrue(m_arm.RunArmToPositionCommand(Math.toRadians(-25)));
/*
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_arm.RunArmToPositionCommand(Math.toRadians(-90)));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_arm.RunArmToPositionCommand(Math.toRadians(0)));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_arm.RunArmToPositionCommand(Math.toRadians(180)));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_arm.RunArmToPositionCommand(Math.toRadians(90)));

*/
                 new Trigger(() -> {
                        return m_driverController.getRightTriggerAxis() > 0;
                }).whileTrue(m_arm.RunArmUpManualSpeedCommand(() -> m_driverController.getRightTriggerAxis()));


                new Trigger(() -> {
                        return m_driverController.getLeftTriggerAxis() > 0;
                }).whileTrue(m_arm.RunArmDownManualSpeedCommand(() -> -m_driverController.getLeftTriggerAxis()));
        }
        

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                
                //Center to the goal
                 new JoystickButton(m_driverController, XboxController.Button.kA.value)
                 //.whileTrue(new CenterToGoalCommand(m_robotDrive, false, true));
                 .whileTrue(new CenterToGoalCommand(m_robotDrive, true));


                


             
                 //Auto drive to trap
                 

                SmartDashboard.putData("Reset Odometry", (Commands.runOnce(() -> m_robotDrive.zeroHeading(), m_robotDrive)));
                

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

       public Command getAutonomousCommand() {
              if (autoChooser != null) {
                SmartDashboard.putString("Auto Running", autoChooser.getSelected().getName());
                return autoChooser.getSelected();
              } else{
                return Commands.runOnce(() -> m_robotDrive.forceStop());
              }
        }

        public void publishAuto() {
                SmartDashboard.putNumber("Pi Theta", m_picam.getPiCamAngle());
        }

}

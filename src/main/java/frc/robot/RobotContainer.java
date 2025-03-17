// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.JoystickCommandsFactory;
import frc.robot.commands.TrajectoryCommandsFactory;
import frc.robot.commands.drive.CenterToGoalCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.DriveToCoordinate;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.field.ScoringPositions.ScoreHeight;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ElevateAssembly.ElevateSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.Elastic;
import frc.robot.utils.ReefButtonBox;
import frc.robot.utils.ScoringPositionSelector;
import frc.robot.utils.Types.GoalType;
import frc.robot.utils.Types.LEDState;
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
        public Limelight m_limelight = new Limelight("limelight-chute");
        public Limelight m_limelight_side = new Limelight("limelight-climb");
        public Limelight m_limelight_three = new Limelight("limelight-elevate");

        XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);
        XboxController m_actionController = new XboxController(IOConstants.kActionControllerPort);
        XboxController m_output_controller = new XboxController(IOConstants.kArduinoOutputPort);
        ReefButtonBox m_buttonbox_controller = new ReefButtonBox(IOConstants.kButtonBoxPort);
        public ScoringPositionSelector m_scoringPositionSelector = new ScoringPositionSelector(m_output_controller);

        public ClimbSubsystem m_climb = new ClimbSubsystem();

        public LEDSubsystem m_ledSystem = new LEDSubsystem();

        private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_limelight, m_limelight_side,m_limelight_three, m_output_controller);// use only 1 limelight for
                                                                                          // driving now since we dont
                                                                                          // have great measurements
                                                                                          // m_limelight_side);

        public ElevateSubsystem m_elevate = new ElevateSubsystem(m_scoringPositionSelector,m_robotDrive, m_ledSystem);

        private final PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);
        private final SendableChooser<Command> autoChooser;

        private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationMetersPerSecondSquared);

        private Boolean m_isAuto = false;



        private void driveWithJoystick(Boolean fieldRelative) {
                // Get the x speed. We are inverting this because Xbox controllers return
                // negative values when we push forward.

                var leftY = m_driverController.getLeftY();
                var leftX = m_driverController.getLeftX();
                var rightX = m_driverController.getRightX();

                if(m_driverController.getLeftStickButton()){
                       leftY /=2; 
                       leftX /=2;
                       System.out.println("Speed Limiting");
                }

                if(m_driverController.getRightStickButton()){
                        rightX /=2;
                }
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
                if(m_driverController.getLeftTriggerAxis()>0){
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
                
                //create named commands for each command in ElevateSubsystem
                NamedCommands.registerCommand("ElevateToL2", m_elevate.GotoScoreL2PositionCommand());
                NamedCommands.registerCommand("ElevateToL3", m_elevate.GotoScoreL3PositionCommand());
                NamedCommands.registerCommand("ElevateToL4", m_elevate.GotoScoreL4PositionCommand());
                NamedCommands.registerCommand("ElevateToScorePrep", m_elevate.PrepScore());
                NamedCommands.registerCommand("ElevateToIntake", m_elevate.PrepForIntakePosition());
                NamedCommands.registerCommand("ElevateToIntakeAndIntake", m_elevate.GoToIntakeAndIntake());
                NamedCommands.registerCommand("ShootOutL1L3", m_elevate.OnlyScore());
                NamedCommands.registerCommand("ShootOutL4", m_elevate.OnlyScoreL4());
                NamedCommands.registerCommand("ShootOutL4NoIntakeReturn", (m_elevate.OnlyScoreL4NoIntakeReturn()));
                NamedCommands.registerCommand("WaitForCoral", m_elevate.WaitForCoral());
                NamedCommands.registerCommand("WaitForCoralOrChute",m_elevate.WaitForCoralOrChute());
                NamedCommands.registerCommand("MoveToL4WhileDrive", m_elevate.AutoIntakeAndL4PositionWhileDriving());
                NamedCommands.registerCommand("IntakeWhileDrive", m_elevate.AutoIntakePositionWhileDriving());

                NamedCommands.registerCommand("ScoreL4EarlyEndNoReturn", m_elevate.ScoreL4CommandEarlyEndNoReturn());

                NamedCommands.registerCommand("Ignore12Oclock", Commands.runOnce(()->m_robotDrive.setIgnore12Oclock(true)));
                NamedCommands.registerCommand("IgnoreMiddleScoring",  Commands.runOnce(()->m_robotDrive.setIgnoreAutoExtras(true)));
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
                                break;
                        case DRIVE:
                                configureButtonBindingsDriveSysID();
                                break;
                        case SHOOTER_LEFT:
                        case SHOOTER_RIGHT:
                                break;
                        case ELEVATOR:
                                configButtonBindingsElevatorSysID();
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

                m_elevate.setArmInitialPosition();

                new Trigger(()->
                {return DriverStation.getMatchTime()  < 25 && !m_isAuto;}).onTrue(Commands.runOnce(()->Elastic.selectTab("ClimbCam")));

                


                
        
        }

        public void resetArmElevator(){
                m_elevate.resetArmElevatorAfterDisable();
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
                m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                                () -> driveWithJoystick(true),
                                m_robotDrive));
        }
    


        private void configButtonBindingsArmSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_elevate.sysIdArmQuasistatic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_elevate.sysIdArmQuasistatic(SysIdRoutine.Direction.kReverse));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_elevate.sysIdArmDynamic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_elevate.sysIdArmDynamic(SysIdRoutine.Direction.kReverse));

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(m_elevate.RunArmToPositionCommand(Math.toRadians(120)));

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .whileTrue(m_elevate.RunArmToPositionCommand(Math.toRadians(-25)));
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
                }).whileTrue(m_elevate.RunArmUpManualSpeedCommand(() -> m_driverController.getRightTriggerAxis()));


                new Trigger(() -> {
                        return m_driverController.getLeftTriggerAxis() > 0;
                }).whileTrue(m_elevate.RunArmDownManualSpeedCommand(() -> -m_driverController.getLeftTriggerAxis()));
        }

        private void configButtonBindingsElevatorSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_elevate.sysIdElevatorQuasistatic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_elevate.sysIdElevatorQuasistatic(SysIdRoutine.Direction.kReverse));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_elevate.sysIdElevatorDynamic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_elevate.sysIdElevatorDynamic(SysIdRoutine.Direction.kReverse));

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(m_elevate.RunElevatorToPositionCommand(10));

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .whileTrue(m_elevate.RunElevatorToPositionCommand(24.5));

                 new Trigger(() -> {
                        return m_driverController.getRightTriggerAxis() > 0;
                }).whileTrue(m_elevate.RunElevatorManualSpeedCommand(() -> m_driverController.getRightTriggerAxis()));


                new Trigger(() -> {
                        return m_driverController.getLeftTriggerAxis() > 0;
                }).whileTrue(m_elevate.RunElevatorManualSpeedCommand(() -> -m_driverController.getLeftTriggerAxis()));
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
                
                m_ledSystem.SetLEDState(LEDState.IN_RANGE);

                //A -> Go to score L2, but dont score
                //B -> Go to score L3, but dont score
                //Y -> Go to and Score L4
                //X -> Go to Prep spot to score L4
                //Right Bumper -> Score for L2 and L3
                //Left Bumper -> Go to intake position and intake
                //Start -> Reset Elevator Encoder
                //Right Trigger -> Elevator Up
                //Left Trigger -> Elevator Down

                //POV Up -> Run Climber Forward
                //POV Down -> Run Climber Reverse

                //POV Right -> Next Cage Pos
                //POV Left -> Previous Cage Pos

                 new JoystickButton(m_driverController, XboxController.Button.kA.value)
                 .whileTrue(m_elevate.GotoScoreL2PositionCommand());

                 new JoystickButton(m_driverController, XboxController.Button.kB.value)
                 .whileTrue(m_elevate.GotoScoreL3PositionCommand());

                 new JoystickButton(m_driverController, XboxController.Button.kY.value)
                 .whileTrue(m_elevate.ScoreL4Command());
                
                 new JoystickButton(m_driverController, XboxController.Button.kX.value)
                        .whileTrue(m_elevate.DriveToCage(m_robotDrive));

                

                 new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                 .whileTrue((TrajectoryCommandsFactory.GoToRightIntake(m_robotDrive).alongWith(m_elevate.PrepForIntakePosition())).raceWith(JoystickCommandsFactory
                 .RumbleControllerTillCancel(m_actionController, RumbleType.kRightRumble)));

                 new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                 .whileTrue((TrajectoryCommandsFactory.GoToLeftIntake(m_robotDrive).alongWith(m_elevate.PrepForIntakePosition())).raceWith(JoystickCommandsFactory
                 .RumbleControllerTillCancel(m_actionController, RumbleType.kLeftRumble)));


                 new JoystickButton(m_driverController, XboxController.Button.kStart.value)
                 .onTrue(m_elevate.ResetElevatorEncoder());

                 new POVButton(m_driverController, 0).whileTrue(m_elevate.PrepForIntakePosition());
                 new POVButton(m_driverController, 180).whileTrue(m_elevate.GoToIntakeAndIntake());

                 new POVButton(m_driverController, 90).whileTrue(m_elevate.CagePosRight());
                 new POVButton(m_driverController, 270).whileTrue(m_elevate.CagePosLeft());
                 new Trigger(() -> {
                        return m_driverController.getRightTriggerAxis() > 0;
                }).whileTrue(m_elevate.OnlyScore());


                SmartDashboard.putData("Reset Odometry", (Commands.runOnce(() -> m_robotDrive.zeroHeading(), m_robotDrive)));

                SmartDashboard.putData("Reset Arm Encoder", (Commands.runOnce(() ->m_elevate.setArmInitialPosition())));
                

                /*******************************************
                 * ACTION CONTROLLER
                 *******************************************/

                new JoystickButton(m_actionController, XboxController.Button.kX.value)
                .whileTrue(m_elevate.ElevateHome());


                new JoystickButton(m_actionController, XboxController.Button.kA.value).and(new JoystickButton(m_actionController, XboxController.Button.kLeftBumper.value)).whileTrue(
                        m_elevate.DriveToNearestScoreCommand(m_robotDrive, true, ScoreHeight.L2).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kLeftRumble))
                );

                new JoystickButton(m_actionController, XboxController.Button.kA.value).and(new JoystickButton(m_actionController, XboxController.Button.kRightBumper.value)).whileTrue(
                        m_elevate.DriveToNearestScoreCommand(m_robotDrive, false, ScoreHeight.L2).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kRightRumble))
                );

                new JoystickButton(m_actionController, XboxController.Button.kB.value).and(new JoystickButton(m_actionController, XboxController.Button.kLeftBumper.value)).whileTrue(
                        m_elevate.DriveToNearestScoreCommand(m_robotDrive, true, ScoreHeight.L3).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kLeftRumble))
                );
                new JoystickButton(m_actionController, XboxController.Button.kB.value).and(new JoystickButton(m_actionController, XboxController.Button.kRightBumper.value)).whileTrue(
                        m_elevate.DriveToNearestScoreCommand(m_robotDrive, false, ScoreHeight.L3).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kRightRumble))
                );

                new JoystickButton(m_actionController, XboxController.Button.kY.value).and(new JoystickButton(m_actionController, XboxController.Button.kLeftBumper.value)).whileTrue(
                        m_elevate.DriveToNearestScoreCommand(m_robotDrive, true, ScoreHeight.L4).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kLeftRumble))
                );
                new JoystickButton(m_actionController, XboxController.Button.kY.value).and(new JoystickButton(m_actionController, XboxController.Button.kRightBumper.value)).whileTrue(
                        m_elevate.DriveToNearestScoreCommand(m_robotDrive, false, ScoreHeight.L4).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kRightRumble))
                );

                new POVButton(m_actionController, 270).onTrue(new InstantCommand (() -> m_scoringPositionSelector.SetPreviousScorePosition()));
                new POVButton(m_actionController, 90).onTrue(new InstantCommand (() -> m_scoringPositionSelector.SetNextScorePosition()));

                new Trigger(() -> {
                        return m_actionController.getRightTriggerAxis() > 0;
                }).and(new JoystickButton(m_actionController, XboxController.Button.kA.value)).whileTrue(
                        m_elevate.DriveToSelectedCommand(m_robotDrive, false, ScoreHeight.L2).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kRightRumble)));
                new Trigger(() -> {
                        return m_actionController.getLeftTriggerAxis() > 0;
                }).and(new JoystickButton(m_actionController, XboxController.Button.kA.value)).whileTrue(
                        m_elevate.DriveToSelectedCommand(m_robotDrive, true, ScoreHeight.L2).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kLeftRumble)));
                new Trigger(() -> {
                        return m_actionController.getRightTriggerAxis() > 0;
                }).and(new JoystickButton(m_actionController, XboxController.Button.kB.value)).whileTrue(
                        m_elevate.DriveToSelectedCommand(m_robotDrive, false, ScoreHeight.L3).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kRightRumble)));
                new Trigger(() -> {
                        return m_actionController.getLeftTriggerAxis() > 0;
                }).and(new JoystickButton(m_actionController, XboxController.Button.kB.value)).whileTrue(
                        m_elevate.DriveToSelectedCommand(m_robotDrive, true, ScoreHeight.L3).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kLeftRumble)));
                new Trigger(() -> {
                        return m_actionController.getRightTriggerAxis() > 0;
                }).and(new JoystickButton(m_actionController, XboxController.Button.kY.value)).whileTrue(
                        m_elevate.DriveToSelectedCommand(m_robotDrive, false, ScoreHeight.L4).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kRightRumble)));
                new Trigger(() -> {
                        return m_actionController.getLeftTriggerAxis() > 0;
                }).and(new JoystickButton(m_actionController, XboxController.Button.kY.value)).whileTrue(
                        m_elevate.DriveToSelectedCommand(m_robotDrive, true, ScoreHeight.L4).raceWith(JoystickCommandsFactory
                        .RumbleControllerTillCancel(m_driverController, RumbleType.kLeftRumble)));
                        

                        new Trigger(()->{
                                return m_actionController.getLeftY() > 0.2;
                        }).whileTrue(m_climb.runClimberToSetpoint(290));    
                        
                         new Trigger(()->{
                                   return m_actionController.getLeftY() < -0.2;
                         }).whileTrue(m_climb.runClimberToSetpoint(150));



                new Trigger(()->{
                        return m_actionController.getRightY() < -0.8;
              }).whileTrue(m_elevate.ClearAlgae(m_robotDrive));

                  new Trigger(()->{
                        return m_actionController.getRightY() > 0.8;
                  }).whileTrue(m_elevate.ClearAlgae(m_robotDrive));
                 

                  new JoystickButton(m_actionController, XboxController.Button.kStart.value)
                  .whileTrue(new DriveToCoordinate(m_robotDrive, 0, 0));
                 


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
        }

        public void setIsAutonomous(boolean isAuto){
                m_isAuto = isAuto;
                m_elevate.setIsAutonomous(m_isAuto);

                if(!isAuto){
                        m_robotDrive.setIgnore12Oclock(false);
                }
               
        }

}

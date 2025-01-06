package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.CenterToGoalCommand;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.field.ScoringPositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcherAssembly.LauncherAssemblySubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.vision.Limelight;
import frc.robot.vision.PiCamera;

public class RobotCommandsFactory {



    public static Command RunFloorIntakeForwardWithShooterIntakeCommand(IntakeSubsystem intake, LauncherAssemblySubsystem launcher) {        
            return new ParallelCommandGroup(launcher.RunShooterBackwardCommand(),intake.RunIntakeForwardCommand(), launcher.RunKickupBackwardCommand() ).until(()->launcher.ArmContainsNote());
    }

    public static Command  RunFloorIntakeWithArmPosition(IntakeSubsystem intake, LauncherAssemblySubsystem launcher){
        return launcher.RunArmToIntakePositionCommand().andThen(RunFloorIntakeForwardWithShooterIntakeCommand(intake, launcher)).andThen(launcher.RunKickupHoldCommand()).andThen(launcher.RunShooterForwardIdle()).andThen(launcher.runArmToFixedAngleCommand().unless(()->!launcher.ArmContainsNote()));
    }

   

    public static Command DriveToTargetWithIntake(DriveSubsystem robot_drive, IntakeSubsystem intake, LauncherAssemblySubsystem launcher, PiCamera picam, double speed, double maxDistance)
    {
        return new DriveToTargetCommand(robot_drive, picam, speed, maxDistance).raceWith(RunFloorIntakeForwardWithShooterIntakeCommand(intake, launcher).handleInterrupt(
            ()->{
                if(launcher.ArmContainsNote()){
                    launcher.RunKickupHold();
                    launcher.RunShooterForwardIdle();
                }
            }
        ));
    }


    public static Command DriveToTargetWithIntakeThenIdle(DriveSubsystem robot_drive, IntakeSubsystem intake, LauncherAssemblySubsystem launcher, PiCamera picam, double speed, double maxDistance)
    {
        return  launcher.RunArmToIntakePositionCommand().andThen(new DriveToTargetCommand(robot_drive, picam, speed, maxDistance).raceWith(RunFloorIntakeForwardWithShooterIntakeCommand(intake, launcher).handleInterrupt(
            ()->{
                if(launcher.ArmContainsNote()){
                    launcher.RunKickupHold();
                    launcher.RunShooterForwardIdle();
                    launcher.runArmToFixedAngle();
                }
            }
        )).andThen(launcher.RunKickupHoldCommand()).andThen(launcher.RunShooterForwardIdle()).andThen(launcher.runArmToFixedAngleCommand().unless(()->!launcher.ArmContainsNote())));

        //return  launcher.RunArmToIntakePositionCommand().andThen(new DriveToTargetCommand(robot_drive, picam, speed, maxDistance).raceWith(RunFloorIntakeForwardWithShooterIntakeCommand(intake, launcher)).andThen(launcher.RunKickupHold()).andThen(launcher.RunShooterForwardIdle()));
    }

        public static Command DriveToTargetWithIntakeThenIdle(DriveSubsystem robot_drive, IntakeSubsystem intake, LauncherAssemblySubsystem launcher, PiCamera picam, DoubleSupplier speed, double maxDistance)
    {
        return  launcher.RunArmToIntakePositionCommand().andThen(new DriveToTargetCommand(robot_drive, picam, speed, maxDistance).raceWith(RunFloorIntakeForwardWithShooterIntakeCommand(intake, launcher).handleInterrupt(
            ()->{
                if(launcher.ArmContainsNote()){
                    launcher.RunKickupHold();
                    launcher.RunShooterForwardIdle();
                    launcher.runArmToFixedAngleCommand().schedule();
                }
            }
        )).andThen(launcher.RunKickupHoldCommand()).andThen(launcher.RunShooterForwardIdle())).andThen(launcher.runArmToFixedAngleCommand().unless(()->!launcher.ArmContainsNote()));

        //return  launcher.RunArmToIntakePositionCommand().andThen(new DriveToTargetCommand(robot_drive, picam, speed, maxDistance).raceWith(RunFloorIntakeForwardWithShooterIntakeCommand(intake, launcher)).andThen(launcher.RunKickupHold()).andThen(launcher.RunShooterForwardIdle()));
    }




     /**
     * Automatically shoot once we get in the range of the target, with target centering active
     * @param launcher
     * @param drive
     * @return
     */
    //public static Command ShootWhenInRange(LauncherAssemblySubsystem launcher, DriveSubsystem drive){
    //    return (launcher.SpoolShooterCommand().alongWith(new CenterToGoalCommand(drive, true)).until(()->MathUtils.distanceToScoringTarget(drive.getPose()) < ScoringPositions.maxDistanceToScoreMeters)).andThen(launcher.RunShooterAndKickupForwardCommand());
    //}


    public static Command ShootWhenInRange(LauncherAssemblySubsystem launcher, DriveSubsystem drive){
        return (launcher.SpoolShooterCommand().alongWith(new CenterToGoalCommand(drive, true)).alongWith(launcher.RunArmToAutoPositionCommandContinuous(drive)).until(
            ()->{
                var distanceToTarget = MathUtils.distanceToScoringTarget(drive.getPose());
                SmartDashboard.putNumber("DistanceToTarget", distanceToTarget);
                
                
                
                return distanceToTarget < ScoringPositions.maxDistanceToScoreMeters && CenterToGoalCommand.checkTurningDoneStatic(drive) && launcher.armIsAtPosition() && launcher.shooterAtSpeedOrFaster();
            })).andThen(launcher.ShootCuzIToldYouYouAreReady());
    }

    

    /**
     * @brief Generate a command that will start where it is, drive to a target
     *        using PiCam vision, intake target,
     *        drive to a scoring position, and score the target.
     * @param robotDrive    subsystem
     * @param picam         pi camera
     * @param limelight     limelight camera
     * @param intake        intake subsystem
     * @param escalator     escalator subsystem
     * @param finalPosition final position to drive to
     * @param height        height to score at
     * @return command
     */

     //TODO
    public static Command generateGrabTargetAndScoreCommmand(DriveSubsystem robotDrive, PiCamera picam,
            Limelight limelight,
            Pose2d finalPosition) {

        return null;
    }


    /**
     * @brief Generate a command that combines the drive to target with a running intake command.
     * @note Command will terminate when the drive to target command terminates, since intake is infinite
     * @param robotDrive
     * @param picam
     * @param intake
     * @return
     */
    public static Command generateDriveToTargetWithIntake(DriveSubsystem robotDrive, PiCamera picam,
            double speed, double maxDistance) {

        return null;
    }


   

  

    /**
     * Generate the command to score to an object a specific position (height), while centering with limelight.
     * @param robotDrive
     * @param escalator
     * @param limelight
     * @param height
     * @return
     */
    public static Command generateScoreCommandWithCentering(DriveSubsystem robotDrive,
            Limelight limelight) {

        return null;
    }

}

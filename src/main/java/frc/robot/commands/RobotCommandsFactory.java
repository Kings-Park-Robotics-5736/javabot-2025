package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.field.ScoringPositions.ScoreHeight;
import frc.robot.field.ScoringPositions.ScoreLocation;
import frc.robot.field.ScoringPositions.ScorePositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.MathUtils;

public class RobotCommandsFactory {

   public static Pose2d GetClosestScorePose(DriveSubsystem robotDrive, ScoreLocation location, ScoreHeight height ){
        ScorePositions closestTarget = MathUtils.getClosestScoringTarget(robotDrive.getPose());
        Pose2d targetPose = TrajectoryCommandsFactory.getPathFinishingPose(closestTarget, location, height);
        return targetPose;
    }

}
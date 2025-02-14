package frc.robot.utils;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.field.ScoringPositions;
import frc.robot.field.ScoringPositions.ScorePositions;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.function.Supplier;

public class MathUtils {


    public static Supplier<String> getClosestScoringTargetSupplier(DriveSubsystem robotDrive, BooleanSupplier left, BooleanSupplier top) {
        return () -> {
            ScorePositions pos = getClosestScoringTarget(robotDrive.getPose());
            return pos.toString() + (left.getAsBoolean() ? "LEFT" : "RIGHT") + (top.getAsBoolean() ? "L4" : "L3");
        };
    }

    public static ScorePositions getClosestScoringTarget(Pose2d robotPose){
        var alliance = DriverStation.getAlliance();
       
        double minDistance = 10000;

        Pose2d[] scoringPositions = {};
        int poseIndex = 0;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            scoringPositions = ScoringPositions.BlueScoringLocations;
        } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            scoringPositions = ScoringPositions.RedScoringLocations;
        }

        for (int i = 0; i < scoringPositions.length; i++){
            Pose2d position = scoringPositions[i];
            double distance = position.getTranslation().getDistance(robotPose.getTranslation());
            if (distance < minDistance){
                minDistance = distance;
                poseIndex = i;
            }
        }


        return ScoringPositions.scorePositionsList[poseIndex];
    }


   
}

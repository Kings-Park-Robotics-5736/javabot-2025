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


    public static String BuildMapKeyString(ScorePositions position, Boolean left, Boolean top) {
        String key = position.toString() + (left ? "LEFT" : "RIGHT") + (top ? "L4" : "L3");
        System.out.println("Driving to key " + key);
        return key;
    }

    public static Supplier<String> getClosestScoringTargetSupplier(DriveSubsystem robotDrive, Boolean left, Boolean top) {
        return () -> {
            ScorePositions pos = getClosestScoringTarget(robotDrive.getPose());
            return BuildMapKeyString(pos, left, top);
        };
    }


    public static double GetDistanceToCloasestScoringTarget(Pose2d robotPose) {
        var alliance = DriverStation.getAlliance();
        double minDistance = 10000;

        Pose2d[] scoringPositions = {};
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            scoringPositions = ScoringPositions.BlueScoringLocations;
        } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            scoringPositions = ScoringPositions.RedScoringLocations;
        }

        for (int i = 0; i < scoringPositions.length; i++) {
            Pose2d position = scoringPositions[i];
            double distance = position.getTranslation().getDistance(robotPose.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
            }
        }

        return minDistance;
    }

    public static Boolean IsWithinRange(Pose2d robotPose){
        return GetDistanceToCloasestScoringTarget(robotPose) < ScoringPositions.maxDistanceToScoreMeters;
    }

    public static Boolean IsAwayFromIntakeStation(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(ScoringPositions.INTAKE_LEFT_BLUE.getTranslation()) > .5 &&
                robotPose.getTranslation().getDistance(ScoringPositions.INTAKE_RIGHT_BLUE.getTranslation()) > .5 &&
                robotPose.getTranslation().getDistance(ScoringPositions.INTAKE_LEFT_RED.getTranslation()) > .5 &&
                robotPose.getTranslation().getDistance(ScoringPositions.INTAKE_RIGHT_RED.getTranslation()) > .5;
    }

    public static Boolean IsCloseToIntakeStation(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(ScoringPositions.INTAKE_LEFT_BLUE.getTranslation()) <1.5 ||
                robotPose.getTranslation().getDistance(ScoringPositions.INTAKE_RIGHT_BLUE.getTranslation()) <1.5 ||
                robotPose.getTranslation().getDistance(ScoringPositions.INTAKE_LEFT_RED.getTranslation()) <1.5 ||
                robotPose.getTranslation().getDistance(ScoringPositions.INTAKE_RIGHT_RED.getTranslation()) <1.5;
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

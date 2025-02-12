package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.field.ScoringPositions;
import frc.robot.field.ScoringPositions.ScorePositions;

public class MathUtils {


    

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

    

    public static Pose2d getRobotPoseAtScoringTarget(ScoringPositions.ScorePositions clockScorePosition, ScoringPositions.ScoreLocation side){

        final double ROBOT_WIDTH_WITH_BUMPERS = Units.inchesToMeters(18.25); //meters
        final double ROBOT_LEFT_SCORE_OFFSET  = Units.inchesToMeters(2.848); //meters
        final double ROBOT_RIGHT_SCORE_OFFSET = Units.inchesToMeters(15.788); //meters

        double offset = side == ScoringPositions.ScoreLocation.LEFT ? ROBOT_LEFT_SCORE_OFFSET : ROBOT_RIGHT_SCORE_OFFSET;

        var alliance = DriverStation.getAlliance();

        Pose2d AprilTagCoord = ScoringPositions.ScorePositionToPose(clockScorePosition, alliance.get());

        double robotAngle = AprilTagCoord.getRotation().getRadians() - Math.toRadians(180);
        double robotX;
        double robotY;
        
        robotX = AprilTagCoord.getX() + Math.cos(AprilTagCoord.getRotation().getRadians())*(ROBOT_WIDTH_WITH_BUMPERS * .5) + Math.sin(AprilTagCoord.getRotation().getRadians()) * offset;

        robotY = AprilTagCoord.getX() + Math.sin(AprilTagCoord.getRotation().getRadians())*(ROBOT_WIDTH_WITH_BUMPERS * .5) + Math.cos(AprilTagCoord.getRotation().getRadians()) * offset;
        
        return new Pose2d( new Translation2d(robotX,robotY), Rotation2d.fromRadians(robotAngle));
    }

   
}

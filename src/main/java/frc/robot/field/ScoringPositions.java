package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public final class ScoringPositions {
    public static final Pose2d kBlueScoringPosition = new Pose2d(new Translation2d(0, 5.55), Rotation2d.fromDegrees(0));
    public static final Pose2d kRedScoringPosition = new Pose2d(new Translation2d(16.5, 5.55), Rotation2d.fromDegrees(0));
    public static final double maxDistanceToScoreMeters = 1;
    public static final double speakerOpeningFromFloorMeters = 2.0574;


    //BLUE scoring april tags
    public static final Pose2d BLUEATSIX = new Pose2d(new Translation2d(3.66, 4.03), Rotation2d.fromDegrees(180));
    public static final Pose2d BLUEATEIGHT = new Pose2d(new Translation2d(4.07, 4.75), Rotation2d.fromDegrees(120));
    public static final Pose2d BLUEATTEN = new Pose2d(new Translation2d(4.90, 4.75), Rotation2d.fromDegrees(60));
    public static final Pose2d BLUEATTWELVE = new Pose2d(new Translation2d(5.32, 4.03), Rotation2d.fromDegrees(0));
    public static final Pose2d BLUEATTWO = new Pose2d(new Translation2d(4.90, 3.31), Rotation2d.fromDegrees(300));
    public static final Pose2d BLUEATFOUR = new Pose2d(new Translation2d(4.07, 3.31), Rotation2d.fromDegrees(240));

    public static final Pose2d BlueScoringLocations[] = {
        BLUEATTWELVE, BLUEATTWO, BLUEATFOUR, BLUEATSIX, BLUEATEIGHT, BLUEATTEN
    };

    //RED april tags
    public static final Pose2d REDATSIX = new Pose2d(new Translation2d(13.89, 4.03), Rotation2d.fromDegrees(0));
    public static final Pose2d REDATEIGHT = new Pose2d(new Translation2d(13.47, 3.31), Rotation2d.fromDegrees(300));
    public static final Pose2d REDATTEN = new Pose2d(new Translation2d(12.64,3.31), Rotation2d.fromDegrees(240));
    public static final Pose2d REDATTWELVE = new Pose2d(new Translation2d(12.23, 4.03), Rotation2d.fromDegrees(180));
    public static final Pose2d REDATTWO = new Pose2d(new Translation2d(12.64, 4.75), Rotation2d.fromDegrees(120));
    public static final Pose2d REDATFOUR = new Pose2d(new Translation2d(13.47, 4.75), Rotation2d.fromDegrees(60));

    public static final Pose2d RedScoringLocations[] = {
        REDATTWELVE, REDATTWO, REDATFOUR, REDATSIX, REDATEIGHT, REDATTEN
    };

    public enum ScorePositions{
        TWELVE,
        TWO,
        FOUR,
        SIX,
        EIGHT,
        TEN
    };

    public static final ScorePositions[] scorePositionsList = {
        ScorePositions.TWELVE, ScorePositions.TWO, ScorePositions.FOUR, ScorePositions.SIX, ScorePositions.EIGHT, ScorePositions.TEN
    };

    public static final String[] scorePositionsListAlphaNames = {
        "HG", "FE", "CD", "AB", "LK", "JI"
    };

    public static final String ScoreClockPositionToAlphaName(ScorePositions pos){
        switch(pos){
            case TWELVE:
                return "HG";
            case TWO:
                return "FE";
            case FOUR:
                return "CD";
            case SIX:
                return "AB";
            case EIGHT:
                return "LK";
            case TEN:
            default:
                return "JI";

        }
    }

    public enum ScoreLocation{
        LEFT,
        RIGHT
    }

    public enum ScoreHeight{
        L1,
        L2,
        L3,
        L4
    }




    public static Pose2d ScorePositionToPose(ScorePositions pos, DriverStation.Alliance color){
        if (color == DriverStation.Alliance.Blue){
            switch(pos){
                case TWELVE:
                    return BLUEATTWELVE;
                case TWO:
                    return BLUEATTWO;
                case FOUR:
                    return BLUEATFOUR;
                case SIX:
                    return BLUEATSIX;
                case EIGHT:
                    return BLUEATEIGHT;
                case TEN:
                default:
                    return BLUEATTEN;

            }
        }else{
            switch(pos){
                case TWELVE:
                    return REDATTWELVE;
                case TWO:
                    return REDATTWO;
                case FOUR:
                    return REDATFOUR;
                case SIX:
                    return REDATSIX;
                case EIGHT:
                    return REDATEIGHT;
                case TEN:
                default:
                    return REDATTEN;

            }
        }

    }
};


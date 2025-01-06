package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class ScoringPositions {
    //todo: get the actual x,y coordinate of the goal
    public static final Pose2d kBlueScoringPosition = new Pose2d(new Translation2d(0, 5.55), Rotation2d.fromDegrees(0));
    public static final Pose2d kRedScoringPosition = new Pose2d(new Translation2d(16.5, 5.55), Rotation2d.fromDegrees(0));
    public static final double maxDistanceToScoreMeters = 4.0;
    public static final double speakerOpeningFromFloorMeters = 2.0574;
}

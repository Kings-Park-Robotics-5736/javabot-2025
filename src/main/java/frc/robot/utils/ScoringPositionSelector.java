package frc.robot.utils;

import java.util.function.Supplier;

import frc.robot.field.ScoringPositions;
import frc.robot.field.ScoringPositions.ScorePositions;

public class ScoringPositionSelector {
    ScorePositions m_scorePosition = ScorePositions.TWELVE;

    public ScoringPositionSelector() {
        m_scorePosition = ScorePositions.TWELVE;
    }

    public void setScorePosition(ScorePositions scorePosition) {
        m_scorePosition = scorePosition;
    }

    public ScorePositions getScorePosition() {
        return m_scorePosition;
    }

    public String getScorePositionString() {
        return m_scorePosition.toString();
    }


    public void SetNextScorePosition() {
        //grab the next score position from ScoringPositions.scorePositionsList
        int currentIndex = m_scorePosition.ordinal();
        int nextIndex = (currentIndex + 1) % ScoringPositions.scorePositionsList.length;
        m_scorePosition = ScoringPositions.scorePositionsList[nextIndex];
    }

    public void SetPreviousScorePosition() {
        //grab the previous score position from ScoringPositions.scorePositionsList
        int currentIndex = m_scorePosition.ordinal();
        int previousIndex = (currentIndex - 1 + ScoringPositions.scorePositionsList.length) % ScoringPositions.scorePositionsList.length;
        m_scorePosition = ScoringPositions.scorePositionsList[previousIndex];
    }
}

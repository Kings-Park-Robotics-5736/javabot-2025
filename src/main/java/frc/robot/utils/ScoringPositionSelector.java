package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.field.ScoringPositions;
import frc.robot.field.ScoringPositions.ScorePositions;


public class ScoringPositionSelector {
    ScorePositions m_scorePosition = ScorePositions.TWELVE;
    XboxController m_output_controller;

    public ScoringPositionSelector(XboxController outputController) {
        m_scorePosition = ScorePositions.TWELVE;
        m_output_controller = outputController;
    }



    public void setScorePosition(ScorePositions scorePosition) {
        m_scorePosition = scorePosition;
        m_output_controller.setRumble(RumbleType.kLeftRumble, ScoringPositions.ScorePositionToRumbleValue(scorePosition));
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
        setScorePosition(ScoringPositions.scorePositionsList[nextIndex]);
    }

    public void SetPreviousScorePosition() {
        //grab the previous score position from ScoringPositions.scorePositionsList
        int currentIndex = m_scorePosition.ordinal();
        int previousIndex = (currentIndex - 1 + ScoringPositions.scorePositionsList.length) % ScoringPositions.scorePositionsList.length;
        setScorePosition(ScoringPositions.scorePositionsList[previousIndex]);
    }
}

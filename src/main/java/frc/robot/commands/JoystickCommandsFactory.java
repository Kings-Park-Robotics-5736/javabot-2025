package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * @brief A factory for creating commands that are used by the joystick
 */
public class JoystickCommandsFactory {

    /**
     * @brief Creates a command that rumbles the controller until canceled
     * @param device the joystick to rumble
     * @return
     */
    public static Command RumbleControllerTillCancel(GenericHID device) {
        return new FunctionalCommand(
                () -> {},
                () -> device.setRumble(RumbleType.kBothRumble, 1),
                (interrupted) -> device.setRumble(RumbleType.kBothRumble, 0),
                () -> false);
    }
}

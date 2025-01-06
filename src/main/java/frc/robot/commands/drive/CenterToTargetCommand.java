package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveToTargetCommandConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.Types.PidConstants;

/**
 * @brief This command centers the robot on the target. It does NOT drive to it
 *        It is useful for a user to manually hold down a button, for instance,
 *        and manually drive to an object
 * 
 * @note Because we want the user to still be able to translate robot, we will
 *       not require robotDrive as a subsystem. Otherwise all joystick controls
 *       will be
 *       locked out. As such, we will manually lock out the rotate joystick.
 */
public abstract class CenterToTargetCommand extends Command {

    protected DriveSubsystem m_drive;
    private boolean m_infinite;

    private static final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
            DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);

    protected final ProfiledPIDController m_controller_theta;

    public CenterToTargetCommand(DriveSubsystem robot_drive, boolean infinite, TrapezoidProfile.Constraints constraints,
            PidConstants pid) {

        this.m_drive = robot_drive;
        this.m_infinite = infinite;

        m_controller_theta = new ProfiledPIDController(
                pid.p, pid.i, pid.d, constraints, Constants.kDt);

        // NOTE - we explicitly don't do the below line; else can't drive while
        // centering
        // addRequirements(m_drive);

    }

    public CenterToTargetCommand(DriveSubsystem robot_drive, boolean infinite) {

        this(robot_drive, infinite, m_constraints, DriveToTargetCommandConstants.kPidValues);

        // NOTE - we explicitly don't do the below line; else can't drive while
        // centering
        // addRequirements(m_drive);

    }

    @Override
    public void initialize() {

        m_drive.setJoystickRotateLockout(true);

        m_controller_theta.reset(m_drive.getHeadingInRadians());
        m_controller_theta.setTolerance(0.01);

    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
        m_drive.setJoystickRotateLockout(false);
        m_drive.setRotateLockoutValue(0);
    }

    @Override
    public boolean isFinished() {
        var done = checkTurningDone();
        System.out.println("Is finished = " + done);
        SmartDashboard.putBoolean("Square to Target?", done);
        return !m_infinite && done;
    }

    protected void stop() {
        m_drive.setRotateLockoutValue(0);
        m_drive.drive(0, 0, 0, false, false);
        System.out.println("----------------Centering to target Done-----------------------");
    }

    // to be implemented by the inherited classes.
    protected abstract boolean checkTurningDone();

    protected void centerOnTarget(double angle, boolean useCameraMeasurement) {
        double rotationVel = DriveCommandsCommon.calculateRotationToTarget(m_drive.getHeadingInRadians(),
                useCameraMeasurement,
                angle, m_controller_theta);
        if (rotationVel > -100) {
            m_drive.setRotateLockoutValue(rotationVel);
            m_drive.drive(0, 0, rotationVel, false, false);
        }

    }
}

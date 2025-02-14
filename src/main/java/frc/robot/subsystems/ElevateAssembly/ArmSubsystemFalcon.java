package frc.robot.subsystems.ElevateAssembly;
import static edu.wpi.first.units.Units.Volts;
import static java.lang.Math.abs;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.TalonUtils;
public class ArmSubsystemFalcon extends SubsystemBase {

    private final TalonFX m_motor;

    private boolean emergencyStop;

    private int staleCounter = 0;
    private double lastPosition = 0;
    private boolean manualControl = true;
    private double armManualOffset = 0;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    /********************************************************
     * SysId variables
     ********************************************************/
   
    private final SysIdRoutine m_sysIdRoutine;
    private double m_globalSetpoint;
    private final VoltageOut m_sysidControl = new VoltageOut(0);
    
    private double usedP;
    private double usedI;
    private double usedD;
    private double usedV;
    private double usedG;
    private TalonFXConfiguration configs;
    private final DoubleSupplier m_armEncoderPositionSupplier;


    public ArmSubsystemFalcon(DoubleSupplier _encoderPosition) {

        m_motor = new TalonFX(ArmConstants.kMotorID, ArmConstants.kCanName);
        m_armEncoderPositionSupplier = _encoderPosition;

        configs = new TalonFXConfiguration();
        configs.Slot0.kP = ArmConstants.kPidValues.p; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = ArmConstants.kPidValues.i; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = ArmConstants.kPidValues.d; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kA = ArmConstants.kFFValues.ka;
        configs.Slot0.kG = ArmConstants.kFFValues.kg;
        configs.Slot0.kV = ArmConstants.kFFValues.kv;
        configs.Slot0.kS = ArmConstants.kFFValues.ks;

        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
        configs.Feedback.SensorToMechanismRatio = 72.73 /  (2 * Math.PI); //convert to arm radians

        configs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.kMaxVelocity;// 2* Math.PI;
        configs.MotionMagic.MotionMagicAcceleration = ArmConstants.kMaxAcceleration; //4* Math.PI; 
        configs.MotionMagic.MotionMagicJerk = ArmConstants.kMaxJerk; //20*Math.PI; 

        if (!TalonUtils.ApplyTalonConfig(m_motor, configs)) { 
            System.out.println("!!!!!ERROR!!!! Could not initialize the + Arm. Restart robot!");
        }

       
        

        m_motor.setNeutralMode(NeutralModeValue.Brake);
       
        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(null, Volts.of(2), null, state->SignalLogger.writeString("arm-state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            m_motor.setControl(m_sysidControl.withOutput(volts));
                        },
                     null,
                        this)); 




        BaseStatusSignal.setUpdateFrequencyForAll(250,m_motor.getPosition(), m_motor.getVelocity(), m_motor.getMotorVoltage());
        m_motor.optimizeBusUtilization();
        SignalLogger.start();
 

        SmartDashboard.putNumber("ARM P", ArmConstants.kPidValues.p);
        SmartDashboard.putNumber("ARM I", ArmConstants.kPidValues.i);
        SmartDashboard.putNumber("ARM D", ArmConstants.kPidValues.d);
        SmartDashboard.putNumber("ARM V", ArmConstants.kFFValues.kv);
        SmartDashboard.putNumber("ARM G", ArmConstants.kFFValues.kg);

        usedP = ArmConstants.kPidValues.p;
        usedI = ArmConstants.kPidValues.i;
        usedD = ArmConstants.kPidValues.d;
        usedV = ArmConstants.kFFValues.kv;
        usedG = ArmConstants.kFFValues.kg;

    }

    public void setInitialPosition(double position){
        m_motor.setPosition(Units.rotationsToRadians(position)); //this is the arm offset value
    }

    public void resetAfterDisable(){
        System.out.println("Reset ARM After Disable!!!");
        manualControl = true;
        setSpeed(0);
    }

    @Override
    public void periodic() {
       
        SmartDashboard.putNumber("Falcon Angle Deg", Math.toDegrees((getFalconAngleRadians())));
        SmartDashboard.putNumber("Falcon Angular Velocity", getFalconAngularVelocityRadiansPerSec());

        var newP = SmartDashboard.getNumber("ARM P", usedP);
        var newI = SmartDashboard.getNumber("ARM I", usedI);
        var newD = SmartDashboard.getNumber("ARM D", usedD);
        var newV = SmartDashboard.getNumber("ARM V", usedV);
        var newG = SmartDashboard.getNumber("ARM G", usedG);
        
        if (newP != usedP) {
            usedP = newP;
            configs.Slot0.kP =(usedP);
            TalonUtils.ApplyTalonConfig(m_motor, configs);
            System.out.println("Using P of " + usedP);
        }

        if (newI != usedI) {
            usedI = newI;
            configs.Slot0.kI = (usedI);
            TalonUtils.ApplyTalonConfig(m_motor, configs);
            System.out.println("Using I of " + usedI);

        }

        if (newD != usedD) {
            usedD = newD;
            configs.Slot0.kD =(usedD);
            TalonUtils.ApplyTalonConfig(m_motor, configs);
            System.out.println("Using D of " + usedD);

        }

        if (newV != usedV && newV != 0) {
            usedV = newV;
            System.out.println("Using V of " + usedV);
            configs.Slot0.kV=(usedV);
            TalonUtils.ApplyTalonConfig(m_motor, configs);
        }

        if (newG != usedG && newG != 0) {
            usedG = newG;
            configs.Slot0.kG =(usedG);
            TalonUtils.ApplyTalonConfig(m_motor, configs);
            System.out.println("Using G of " + usedG);
        }

        if(!manualControl){
            RunArmToPos();
        }
           
        if(getFalconAngularVelocityRadiansPerSec()<0.001 && getFalconAngleRadians() > Math.toRadians(120) && Math.abs(getFalconAngleRadians() - Units.rotationsToRadians(m_armEncoderPositionSupplier.getAsDouble())) > Math.toRadians(1.5)){
            setInitialPosition(m_armEncoderPositionSupplier.getAsDouble());
        }
        
    }

     public void RunArmToPos() {

        m_motor.setControl(m_request.withPosition(m_globalSetpoint + armManualOffset));

       
        SmartDashboard.putNumber("Arm Position Eror", Math.toDegrees(m_globalSetpoint - getArmAngleRadians()));
        SmartDashboard.putNumber("Arm Offset Manual", Math.toDegrees(armManualOffset));
        SmartDashboard.putNumber("Arm Global Setpoint ", Math.toDegrees(m_globalSetpoint));
        
    }
   

    /**
     * 
     * Set the speed of the intake motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        m_motor.set(speed);
    }

    /**
     * Stop the motor
     */
    public void StopArm() {
        System.out.println("Arm Stopping");
        setSpeed(0);
    }

    /*****************************
     * Getters for arm position
     *****************************/
    public double getArmAngleRadians() {
        return getFalconAngleRadians();
    }

    public double getFalconAngleRadians() {
        return (m_motor.getPosition().refresh().getValueAsDouble());
    }


    public double getFalconAngularVelocityRadiansPerSec() {
        return m_motor.getVelocity().refresh().getValueAsDouble();
    }

    /**
     * @brief Checks if the arm has reached its target
     * @return true if the arm has reached its target, false otherwise
     */
    public Boolean armReachedTarget() {

        // check if the arm has stalled and is no longer moving
        // if it hasn't moved (defined by encoder change less than kDiffThreshold),
        // increment the stale counter
        // if it has moved, reset the stale counter
        if (Math.abs(getArmAngleRadians() - lastPosition) < ArmConstants.kDiffThreshold) {
            staleCounter++;
        } else {
            staleCounter = 0;
        }
        lastPosition = getArmAngleRadians();

        // calculate the difference between the current position and the motion profile
        // final position
        double delta = Math.abs(getArmAngleRadians() - m_globalSetpoint);

        // we say that the elevator has reached its target if it is within
        // kDiffThreshold of the target,
        // or if it has been within a looser kStaleTolerance for kStaleThreshold cycles
        return delta < ArmConstants.kDiffThreshold
                || (delta < ArmConstants.kStaleTolerance && staleCounter > ArmConstants.kStaleThreshold);
    }

    public boolean armIsDown(){
        return  Math.abs(getArmAngleRadians() - ArmConstants.intakeAngle) < Math.toRadians(1.5);
    }

    private Boolean isFinished() {
        System.out.println("emergency stop " + emergencyStop + ", " + "arm target: " + armReachedTarget());

        var isFinished = emergencyStop || armReachedTarget();
        return isFinished;
    }




    private double sanitizePositionSetpoint(double setpoint){
        if (setpoint > ArmConstants.kLimits.high){
            setpoint = ArmConstants.kLimits.high;
        }
        if(setpoint < ArmConstants.kLimits.low){
            setpoint = ArmConstants.kLimits.low;
        }
        return setpoint;
    }


    public void RunArmToPosition(double setpoint){
        double sanitizedSetpoint = sanitizePositionSetpoint(setpoint);
        System.out.println("-----------------Starting Arm to position " + sanitizedSetpoint + " --------------");
      
        m_globalSetpoint = sanitizedSetpoint;
        manualControl = false;
    }
    /**
     * 
     * @param setpoint the desired arm position IN RADIANS
     * @note When FinishWhenAtTargetSpeed is true, the StopShooter() should not be
     *       called when the command finishes.
     * @return
     */

    public Command RunArmToPositionCommand(double setpoint) {
        return new FunctionalCommand(
                () -> {
                    RunArmToPosition(setpoint);
                },
                () -> {},
                (interrupted) -> {
                    emergencyStop = false;
                },
                () -> {
                    return isFinished();
                }, this);
    }

    public void UpdateAngleManually(double diff){
        armManualOffset +=diff;
    }

    public Command RunArmUpManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Manual Speed Arm Up Starting--------------");
                    manualControl = true;
                },
                () -> {
                    setSpeed(getSpeed.getAsDouble());
                },
                (interrupted) -> {
                    StopArm();
                },
                () -> {
                    return false;
                }, this);
    }

    public Command RunArmDownManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Manual Speed Arm Down Starting--------------");
                    manualControl = true;
                },
                () -> {
                    setSpeed(getSpeed.getAsDouble());
                },
                (interrupted) -> {
                    StopArm();
                },
                () -> {
                    return false;
                }, this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); 
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); 
    }

}
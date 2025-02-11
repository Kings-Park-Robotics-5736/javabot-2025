package frc.robot.subsystems.ElevateAssembly;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.TalonUtils;
import frc.robot.utils.MathUtils;
public class ElevatorSubsystemKraken extends SubsystemBase{

    private final TalonFX m_leader;
    private final TalonFX m_follower;

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
    
    public ElevatorSubsystemKraken(){
        m_leader = new TalonFX(ElevatorConstants.kLeaderDeviceId, ElevatorConstants.kCanName);
        m_follower = new TalonFX(ElevatorConstants.kFollowerDeviceId, ElevatorConstants.kCanName);
        configs = new TalonFXConfiguration();
        configs.Slot0.kP = ElevatorConstants.kPidValues.p; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = ElevatorConstants.kPidValues.i; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = ElevatorConstants.kPidValues.d; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kA = ElevatorConstants.kFFValues.ka;
        configs.Slot0.kG = ElevatorConstants.kFFValues.kg;
        configs.Slot0.kV = ArmConstants.kFFValues.kv;
        configs.Slot0.kS = ElevatorConstants.kFFValues.ks;
        
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
        configs.Feedback.SensorToMechanismRatio = 72.73 /  (2 * Math.PI); //convert to arm radians -idk if we need this in elevator, though me must account for throughbore gearbox

        configs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kMaxVelocity;// 2* Math.PI;
        configs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kMaxAcceleration; //4* Math.PI; 
        configs.MotionMagic.MotionMagicJerk = ElevatorConstants.kMaxJerk; //20*Math.PI; 

        if (!TalonUtils.ApplyTalonConfig(m_leader, configs)||!TalonUtils.ApplyTalonConfig(m_follower, configs)) { 
            System.out.println("!!!!!ERROR!!!! Could not initialize the Elevator. Restart robot!");
        }

        
       m_follower.setControl(new Follower(m_leader.getDeviceID(), false));





m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(null, Volts.of(2), null, state->SignalLogger.writeString("Elevator-state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            m_leader.setControl(m_sysidControl.withOutput(volts));
                        },
                     null,
                        this)); 


        BaseStatusSignal.setUpdateFrequencyForAll(250,m_leader.getPosition(), m_leader.getVelocity(), m_leader.getMotorVoltage());
        m_leader.optimizeBusUtilization();
        SignalLogger.start();

        SmartDashboard.putNumber("Elevator P", ElevatorConstants.kPidValues.p);
        SmartDashboard.putNumber("Elevator I", ElevatorConstants.kPidValues.i);
        SmartDashboard.putNumber("Elevator D", ElevatorConstants.kPidValues.d);
        SmartDashboard.putNumber("Elevator V", ElevatorConstants.kFFValues.kv);
        SmartDashboard.putNumber("Elevator G", ElevatorConstants.kFFValues.kg);

        usedP = ElevatorConstants.kPidValues.p;
        usedI = ElevatorConstants.kPidValues.i;
        usedD = ElevatorConstants.kPidValues.d;
        usedV = ElevatorConstants.kFFValues.kv;
        usedG = ElevatorConstants.kFFValues.kg;

    }

    public void resetAfterDisable(){
        System.out.println("Reset After Disable!!!");
        manualControl = true;
        //setSpeed(0);
    }


















    }
    


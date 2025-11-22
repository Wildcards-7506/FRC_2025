
//This will be the control for the claw on the robot
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.CraneState;

public class Crane extends SubsystemBase {
    public CraneState craneState = CraneState.STOW; // stow is the starting configuration
    /** Degree of angleMargin so that the crane can progress to the next position. */
    public static boolean climbMode = false;
    public boolean runSetpoint = false;
    
    // Wrist
    private final SparkMax wristMotor;
    private final SparkMaxConfig wristConfig;
    public final SparkClosedLoopController wristPID;
    public double wristSetpoint;
    
    // Elbow
    private final SparkMax elbowMotor;
    private final SparkMaxConfig elbowConfig;
    public final SparkClosedLoopController elbowPID;
    public double elbowSetpoint;
    
    //Extender
    private final SparkMax extenderMotor;
    private final SparkMaxConfig extenderConfig;
    public final SparkClosedLoopController extenderPID;
    public double extenderSetpoint;

    //Sucker
    private final SparkMax suckerMotor;
    private final SparkMaxConfig suckerConfig;
    public double suckerSetpoint;
    
    public Crane() {
        wristMotor = new SparkMax(CANIDS.WRIST, MotorType.kBrushless);
        wristConfig = new SparkMaxConfig();
        wristPID = wristMotor.getClosedLoopController();

        elbowMotor = new SparkMax(CANIDS.ELBOW, MotorType.kBrushless);
        elbowConfig = new SparkMaxConfig();
        elbowPID = elbowMotor.getClosedLoopController();

        extenderMotor = new SparkMax(CANIDS.EXTENDER, MotorType.kBrushless);
        extenderConfig = new SparkMaxConfig();
        extenderPID = extenderMotor.getClosedLoopController();

        suckerMotor = new SparkMax(CANIDS.SUCKER, MotorType.kBrushless);
        suckerConfig = new SparkMaxConfig();

        wristConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        wristConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(CraneConstants.kWristCeiling)
            .reverseSoftLimit(CraneConstants.kWristHardDeck);
        wristConfig.encoder
            .positionConversionFactor(CraneConstants.kWristEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kWristEncoderDistancePerPulse);
        wristConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // .pid(0.005, 0.000003, 0.1);
            .pid(0.005, 0.0, 0.1);
            
        elbowConfig
            .smartCurrentLimit(80)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        elbowConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(CraneConstants.kElbowCeiling + 2)
            .reverseSoftLimit(CraneConstants.kElbowHardDeck - 2);
        elbowConfig.encoder
            .positionConversionFactor(CraneConstants.kElbowEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kElbowEncoderDistancePerPulse);
        elbowConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.007, 0.0, 0.05);
            
        extenderConfig
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        extenderConfig.encoder
            .positionConversionFactor(CraneConstants.kExtenderEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kExtenderEncoderDistancePerPulse);
        extenderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.005, 0.0, 0.1);
            
        suckerConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elbowMotor.configure(elbowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extenderMotor.configure(extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        suckerMotor.configure(suckerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
        
    /**
     * This method spins the sucker motor based on voltage and the direction 
     * provided by a sign (e.g. -12).
     * 
     * @param volts The volts to spin the sucker motor, max around (+/-) 12 volts.
     */
    public void spinSucker(double volts) {
        suckerMotor.setVoltage(volts);
    }

    /**
     * Sets the angle of the wrist, shaft CCW+.
     * 
     * @param setPoint The desired angle of the wrist in degrees
     */
    public void setWristPosition(double setPoint) {
        wristPID.setReference(setPoint, ControlType.kPosition);
    }

    /**
     * Sets the angle of the elbow, shaft CW+.
     * 
     * @param setPoint The desired angle of the elbow in degrees
     */
    public void setElbowPosition(double setPoint) {
        elbowPID.setReference(setPoint, ControlType.kPosition);
    }

    /**
     * Sets the extension of the extender, setpoint and actual position are flipped.
     * Full extension is setpoint = ceiling, motor = 0.
     * Full retraction is setpoint = 0, motor = ceiling.
     * 
     * @param setPoint The desired extension of the extender in inches
     */
    public void setExtenderPosition(double setPoint) {
        extenderPID.setReference(setPoint, ControlType.kPosition);
    }

    public void neutralExtend(){
        extenderMotor.stopMotor();
    }

    /** Returns the current angle of the elbow in degrees, CW+. */
    public double getElbowPosition() {
        return elbowMotor.getEncoder().getPosition();
    }

    //Returns the speed of elbow rotation
    public double getElbowVelocity(){
        return elbowMotor.getEncoder().getVelocity();
    }

    /** Returns the extension of the extender in inches, 0 = retracted, ceiling = extended, CCW+. */
    public double getExtenderPosition() {
        return extenderMotor.getEncoder().getPosition();
    }

    /** Returns the angle of the wrist in degrees, CCW+. */
    public double getWristPosition() {
        return wristMotor.getEncoder().getPosition();
    }

    /** Returns the angle of the sucker in degrees, CCW+. */
    public double getSuckerPosition() {
        return suckerMotor.getEncoder().getPosition();
    }

    public double getSuckerCurrent() {
        return suckerMotor.getOutputCurrent();
    }
}

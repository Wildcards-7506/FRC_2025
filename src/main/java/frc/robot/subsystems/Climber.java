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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    public boolean onClimberControl = false;
    
    // Climber
    private final SparkMax climberMotor1;
    // private final SparkMax climberMotor2;
    private final SparkMaxConfig climberConfig;
    public double climberSetpoint;
    
    private final SparkMax tensionerMotor;
    private final SparkMaxConfig tensionerConfig;

    private final SparkMax winchMotor;
    private final SparkMaxConfig winchConfig;
    private final SparkClosedLoopController winchPID;
    public double winchSetpoint;

    public Climber() {

        climberMotor1 = new SparkMax(CANIDS.CLIMBER1, MotorType.kBrushless);
        // climberMotor2 = new SparkMax(CANIDS.CLIMBER2, MotorType.kBrushless);
        climberConfig = new SparkMaxConfig();

        tensionerMotor = new SparkMax(CANIDS.TENSIONER, MotorType.kBrushless);
        tensionerConfig = new SparkMaxConfig();
        
        winchMotor = new SparkMax(CANIDS.WINCH, MotorType.kBrushless);
        winchConfig = new SparkMaxConfig();
        winchPID = winchMotor.getClosedLoopController();

        climberConfig
            .smartCurrentLimit(100)
            .idleMode(IdleMode.kBrake);
        climberConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(ClimberConstants.kClimberCeiling) 
            .reverseSoftLimit(ClimberConstants.kClimberHardDeck);
        climberConfig.encoder
            .positionConversionFactor(ClimberConstants.kClimberEncoderDistancePerPulse);
        climberConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            
        climberMotor1.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // climberMotor2.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        tensionerConfig
            .smartCurrentLimit(20)
            .idleMode(IdleMode.kCoast);
            
        tensionerMotor.configure(tensionerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        winchConfig
            .smartCurrentLimit(80)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        winchConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(ClimberConstants.kWinchCeiling)
            .reverseSoftLimit(ClimberConstants.kWinchHardDeck);
        winchConfig.encoder
            .positionConversionFactor(ClimberConstants.kWinchEncoderDistancePerPulse)
            .velocityConversionFactor(ClimberConstants.kWinchEncoderDistancePerPulse);
        winchConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.005, 0.000003, 0.1);
            
        winchMotor.configure(winchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climberSetpoint = getClimberPosition();
        winchSetpoint = getWinchPosition();
    }

    public void setClimberVoltage(double volts) {
        climberMotor1.setVoltage(volts);
        // climberMotor2.setVoltage(volts);
    }

    /**
     * Moves tensioner
     * 
     * @param volts Positive values keep tension.
     */
    public void setTensionerVoltage(double volts) {
        tensionerMotor.setVoltage(volts);
    }

    public void setWinchPosition(double setPoint) {
        winchSetpoint = filterSetPoint(setPoint, 
                                       ClimberConstants.kWinchHardDeck, 
                                       ClimberConstants.kWinchCeiling);
        winchPID.setReference(setPoint, ControlType.kPosition);
    }
    
    /**
     * Returns the ceiling if the setpoint is above it, or the hard deck if the setpoint is below it.
     * Otherwise, returns the setpoint.
     * 
     * @param setPoint The desired state of the crane.
     * @param hardDeck The hard deck of the crane.
     */
    private double filterSetPoint(double setPoint, double hardDeck, double ceiling) {
        if(setPoint < hardDeck)
            setPoint = hardDeck;
        if(setPoint > ceiling)
            setPoint = ceiling;
        return setPoint;
    }

    public double getClimberPosition() {
        return climberMotor1.getEncoder().getPosition();
    }

    public double getWinchPosition() {
        return winchMotor.getEncoder().getPosition();
    }
}
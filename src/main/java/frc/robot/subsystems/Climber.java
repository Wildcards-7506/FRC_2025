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
    private final SparkMax anchorMotor;
    private final SparkMaxConfig anchorConfig;
    
    private final SparkMax pivotMotor;
    private final SparkMaxConfig pivotConfig;

    private final SparkMax winchMotor;
    private final SparkMaxConfig winchConfig;
    private final SparkClosedLoopController winchPID;

    public Climber() {

        anchorMotor = new SparkMax(CANIDS.ANCHOR, MotorType.kBrushless);
        anchorConfig = new SparkMaxConfig();

        pivotMotor = new SparkMax(CANIDS.PIVOT, MotorType.kBrushless);
        pivotConfig = new SparkMaxConfig();
        
        winchMotor = new SparkMax(CANIDS.WINCH, MotorType.kBrushless);
        winchConfig = new SparkMaxConfig();
        winchPID = winchMotor.getClosedLoopController();

        anchorConfig
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kBrake);
        anchorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(ClimberConstants.kAnchorMax) 
            .reverseSoftLimit(ClimberConstants.kAnchorMin);
        anchorConfig.encoder
            .positionConversionFactor(ClimberConstants.kAnchorEncoderDistancePerPulse);
        anchorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            
        pivotConfig
            .smartCurrentLimit(20)
            .idleMode(IdleMode.kCoast);
        
        winchConfig
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kBrake);
        winchConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(ClimberConstants.kWinchMax)
            .reverseSoftLimit(ClimberConstants.kWinchMin);
        winchConfig.encoder
            .positionConversionFactor(ClimberConstants.kWinchEncoderDistancePerPulse)
            .velocityConversionFactor(ClimberConstants.kWinchEncoderDistancePerPulse);
        winchConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.05, 0.0, 0.1);
          
        anchorMotor.configure(anchorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        winchMotor.configure(winchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setAnchorVoltage(double volts) {
        anchorMotor.setVoltage(volts);
    }

    public void setPivotVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }

    public void setWinchPosition(double setPoint) {
        winchPID.setReference(setPoint, ControlType.kPosition);
    }

    public double getAnchorPosition() {
        return anchorMotor.getEncoder().getPosition();
    }

    public double getWinchPosition() {
        return winchMotor.getEncoder().getPosition();
    }

    public void testModeConfig(){
        winchConfig
            .smartCurrentLimit(100)
            .idleMode(IdleMode.kBrake);
        winchConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);
        winchConfig.encoder
            .positionConversionFactor(ClimberConstants.kWinchEncoderDistancePerPulse)
            .velocityConversionFactor(ClimberConstants.kWinchEncoderDistancePerPulse);
        winchConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.05, 0.0, 0.1);
            
        winchMotor.configure(winchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
    private final SparkMax climberMotor2;
    private final SparkMaxConfig climberConfig;
    public double climberSetpoint;

    public Climber() {

        climberMotor1 = new SparkMax(CANIDS.CLIMBER1, MotorType.kBrushless);
        climberMotor2 = new SparkMax(CANIDS.CLIMBER2, MotorType.kBrushless);
        climberConfig = new SparkMaxConfig();

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
        climberMotor2.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climberSetpoint = getClimberPosition();
    }

    public void setClimberVoltage(double volts) {
        climberMotor1.setVoltage(volts);
        climberMotor2.setVoltage(volts);
    }

    public double getClimberPosition() {
        return climberMotor1.getEncoder().getPosition();
    }
}
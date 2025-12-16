
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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.CraneConstants;

public class Crane extends SubsystemBase {   

    // Crane States - Set States here, then pass the states around the rest of code
    public enum CraneState {
        CLIMB(131.0, 6.25, 30.0),
        STOW_PREP(13.0, 9, 0.0),
        STOW(3.0, 25.5, 0.0),
        STATION(25.0, 9.4, 53.0),
        SHELF(32.0, 10.7, 105.0),
        LOW(73.6, 9.0, 185.0),
        MID(97.6, 15.0, 200.0),
        HIGH_PREP(125.5, 9.0, 30.0),
        HIGH(125.5, 26.0, 30.0),
        ALGAE_HIGH(73.6, 10.5, 150.0),
        ALGAE_LOW(25.0, 10.5, 100.0);

        public double boomAngle;
        public double extension;
        public double wristAngle;

        CraneState(double boomAngle, double extension, double wristAngle){
            this.boomAngle = boomAngle;
            this.extension = extension;
            this.wristAngle = wristAngle;
        }
    }
    // Wrist Rotator
    private final SparkMax wristRotatorMotor;
    private final SparkMaxConfig wristRotatorConfig;
    public final SparkClosedLoopController wristRotatorPID;
    
    // Boom Rotator
    private final SparkMax boomRotatorMotor;
    private final SparkMaxConfig boomRotatorConfig;
    public final SparkClosedLoopController boomRotatorPID;
    
    //Extender
    private final SparkMax extenderMotor;
    private final SparkMaxConfig extenderConfig;
    public final SparkClosedLoopController extenderPID;

    //Intake
    private final SparkMax intakeMotor;
    private final SparkMaxConfig intakeConfig;
    
    public Crane() {
        wristRotatorMotor = new SparkMax(CANIDS.WRIST, MotorType.kBrushless);
        wristRotatorConfig = new SparkMaxConfig();
        wristRotatorPID = wristRotatorMotor.getClosedLoopController();

        boomRotatorMotor = new SparkMax(CANIDS.BOOM, MotorType.kBrushless);
        boomRotatorConfig = new SparkMaxConfig();
        boomRotatorPID = boomRotatorMotor.getClosedLoopController();

        extenderMotor = new SparkMax(CANIDS.EXTENDER, MotorType.kBrushless);
        extenderConfig = new SparkMaxConfig();
        extenderPID = extenderMotor.getClosedLoopController();

        intakeMotor = new SparkMax(CANIDS.INTAKE, MotorType.kBrushless);
        intakeConfig = new SparkMaxConfig();

        wristRotatorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        wristRotatorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(CraneConstants.kWristRotatorMax)
            .reverseSoftLimit(CraneConstants.kWristRotatorMin);
        wristRotatorConfig.encoder
            .positionConversionFactor(CraneConstants.kWristRotatorEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kWristRotatorEncoderDistancePerPulse);
        wristRotatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.005, 0.0, 0.1);
            
        boomRotatorConfig
            .smartCurrentLimit(80)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        boomRotatorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(CraneConstants.kBoomRotatorMax)
            .reverseSoftLimit(CraneConstants.kBoomRotatorMin);
        boomRotatorConfig.encoder
            .positionConversionFactor(CraneConstants.kBoomRotatorEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kBoomRotatorEncoderDistancePerPulse);
        boomRotatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.007, 0.0, 0.05);
            
        extenderConfig
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
            extenderConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(CraneConstants.kExtenderMax)
            .reverseSoftLimit(CraneConstants.kExtenderMin);
        extenderConfig.encoder
            .positionConversionFactor(CraneConstants.kExtenderEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kExtenderEncoderDistancePerPulse);
        extenderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.005, 0.0, 0.1);
            
        intakeConfig
            .smartCurrentLimit(20)
            .idleMode(IdleMode.kBrake);
        
        wristRotatorMotor.configure(wristRotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        boomRotatorMotor.configure(boomRotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extenderMotor.configure(extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
        
    /**
     * This method spins the intake motor based on voltage and the direction 
     * provided by a sign (e.g. -12).
     * 
     * @param volts The volts to spin the intake motor, max around (+/-) 12 volts.
     */
    public void spinIntake(double volts) {
        intakeMotor.setVoltage(volts);
    }

    /**
     * Sets the angle of the wristRotator, shaft CCW+.
     * @param setPoint The desired angle of the wristRotator in degrees
     */
    public void setWristRotatorPosition(double setPoint) {
        wristRotatorPID.setReference(setPoint, ControlType.kPosition);
    }

    /**
     * Sets the angle of the boomRotator, shaft CW+.
     * @param setPoint The desired angle of the boomRotator in degrees
     */
    public void setBoomRotatorPosition(double setPoint) {
        boomRotatorPID.setReference(setPoint, ControlType.kPosition);
    }

    /**
     * Sets the extension of the extender, setpoint and actual position are flipped.
     * Full extension is setpoint = Max Extension, motor = 0.
     * Full retraction is setpoint = 0, motor = Max Extension.
     * 
     * @param setPoint The desired extension of the extender in inches
     */
    public void setExtenderPosition(double setPoint) {
        extenderPID.setReference(CraneConstants.kExtenderMax - setPoint, ControlType.kPosition);
    }

    //Stops extension motor output 
    public void neutralExtend(){
        extenderMotor.stopMotor();
    }

    /** Returns the current angle of the boom in degrees, CW+. */
    public double getBoomPosition() {
        return boomRotatorMotor.getEncoder().getPosition();
    }

    //Returns the speed of boom rotation
    public double getBoomVelocity(){
        return boomRotatorMotor.getEncoder().getVelocity();
    }

    /** Returns the extension of the extender in inches, CCW+. */
    public double getExtensionPosition() {
        return extenderMotor.getEncoder().getPosition();
    }

    //Returns the speed of extension
    public double getExtensionVelocity(){
        return extenderMotor.getEncoder().getVelocity();
    }

    /** Returns the angle of the wrist in degrees, CCW+. */
    public double getWristPosition() {
        return wristRotatorMotor.getEncoder().getPosition();
    }

    //Returns the speed of wrist rotation
    public double getWristVelocity(){
        return wristRotatorMotor.getEncoder().getVelocity();
    }

    //Returns the current draw of the intake motor
    public double getIntakeCurrent() {
        return intakeMotor.getOutputCurrent();
    }
}

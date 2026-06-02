package frc.robot.Motors;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.TurretConstants;
import frc.robot.Interfaces.TurretIO;

public class TurretIOSpark implements TurretIO {

    private final SparkMax turretMotor;
    private final RelativeEncoder turretEncoder;
    private Rotation2d currentTargetAngle = new Rotation2d();

    public TurretIOSpark(){
        turretMotor = new SparkMax(TurretConstants.kMotorId, SparkMax.MotorType.kBrushless);
        turretEncoder = turretMotor.getEncoder();
        var config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(TurretConstants.kPositionFactor);
        config.encoder.velocityConversionFactor(TurretConstants.kVelocityFactor);
        config
            .idleMode(IdleMode.kBrake)
            .inverted(TurretConstants.kMotorInverted)
            .smartCurrentLimit(TurretConstants.kCurrentLimit)
            .voltageCompensation(TurretConstants.kMaxVolts);
        config.softLimit
            .forwardSoftLimit(TurretConstants.kUpperLimit)
            .reverseSoftLimit(TurretConstants.kLowerLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(false);
        config.closedLoop
            .pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD)
            .outputRange(TurretConstants.kMinOutput,TurretConstants.kMaxOutput);
        config.closedLoop.feedForward
            .kS(TurretConstants.kS)
            .kV(TurretConstants.kV)
            .kA(TurretConstants.kA);
        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.angle = Rotation2d.fromRotations(-turretEncoder.getPosition());
        inputs.rps = turretEncoder.getVelocity();
        inputs.voltage = turretMotor.getAppliedOutput()*turretMotor.getBusVoltage();
        inputs.current = turretMotor.getOutputCurrent();
        inputs.targetAngle = this.currentTargetAngle;
    }

    @Override
    public void setVoltage(double voltage){
        turretMotor.setVoltage(voltage);
    }

    @Override
    public void setDutyCycle(double percentage){
        turretMotor.set(percentage);  
    }

    @Override
    public void setPosition(Rotation2d angle){
        this.currentTargetAngle = angle;
        turretMotor.getClosedLoopController().setSetpoint(angle.getRotations(), ControlType.kPosition);
    }

    @Override
    public void setPositionWithFF(Rotation2d angle, double feedforward) {
        turretMotor.getClosedLoopController().setSetpoint(
            angle.getRotations(),ControlType.kPosition, ClosedLoopSlot.kSlot0,feedforward, ArbFFUnits.kVoltage);
    }
    @Override
    public void resetEncoder(){
        turretEncoder.setPosition(0);
    }

    @Override
    public void stop(){
        turretMotor.stopMotor();
    }
    
}

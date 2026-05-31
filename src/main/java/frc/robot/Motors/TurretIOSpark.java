package frc.robot.Motors;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.TurretConstants;
import frc.robot.Interfaces.TurretIO;
import edu.wpi.first.math.util.Units;

public class TurretIOSpark implements TurretIO {

    private final SparkMax turretMotor;
    private final RelativeEncoder turretEncoder;

    public TurretIOSpark(){
        turretMotor = new SparkMax(TurretConstants.kMotorId, SparkMax.MotorType.kBrushless);
        turretEncoder = turretMotor.getEncoder();
        var config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(TurretConstants.kPositionFactor);
        config.encoder.velocityConversionFactor(TurretConstants.kVelocityFactor);
        config.softLimit
            .forwardSoftLimit(TurretConstants.kUpperLimit)
            .reverseSoftLimit(TurretConstants.kLowerLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(false);
        config.closedLoop
            .pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.angle = Rotation2d.fromRotations(turretEncoder.getPosition());
        inputs.rps = turretEncoder.getVelocity();
        inputs.voltage = turretMotor.getAppliedOutput()*turretMotor.getBusVoltage();
        inputs.current = turretMotor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double voltage){
        turretMotor.setVoltage(voltage);
    }

    @Override
    public void setDutyCycle(double percentage){
        turretMotor.getClosedLoopController().setReference(percentage, SparkMax.ControlType.kDutyCycle);   }

    @Override
    public void setPosition(Rotation2d angle){
        turretMotor.getClosedLoopController().setReference(angle.getRotations(), SparkMax.ControlType.kPosition);
    }

    @Override
    public void setPositionWithFF(Rotation2d angle, double feedforward) {
        turretMotor.getClosedLoopController().setReference(
            angle.getRotations(),          //  Setpoint
            ControlType.kPosition,         //  Control
            ClosedLoopSlot.kSlot0,         //  Slot 
            feedforward,                   //  Valor del Feedforward
            ArbFFUnits.kVoltage            //  Unidades del Feedforward
        );
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

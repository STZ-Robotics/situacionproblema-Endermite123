package frc.robot.Simulation;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.TurretConstants;
import frc.robot.Interfaces.TurretIO;

public class TurretSim implements TurretIO {

    final SingleJointedArmSim turretSim;
    final ProfiledPIDController turrerController;
    double appliedVoltage = 0;
    boolean isClosedLoop = false;
    double currentTargetAngle =0;

    public TurretSim(){
        turretSim = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            TurretConstants.kGearRatio, 
            SingleJointedArmSim.estimateMOI(
                TurretConstants.kRadius.in(Meters),
                TurretConstants.kMass.in(Kilograms)
            ), 
            TurretConstants.kRadius.in(Meters), 
            Units.degreesToRadians(TurretConstants.kLowerLimit), 
            Units.degreesToRadians(TurretConstants.kUpperLimit), 
            false,
            Units.degreesToRadians(0)
        );
        turrerController = new ProfiledPIDController(20, 0, 0, 
            new TrapezoidProfile.Constraints(2,4)); 
    }

    @Override
    public void updateInputs(TurretInputs inputs) {

        if (isClosedLoop){
            appliedVoltage = turrerController.calculate(Units.radiansToRotations(turretSim.getAngleRads()), currentTargetAngle);
        }
        appliedVoltage = MathUtil.clamp(appliedVoltage,-12,12);
        turretSim.setInputVoltage(appliedVoltage);
        turretSim.update(0.02);
        inputs.angle = Rotation2d.fromRadians(turretSim.getAngleRads());
        inputs.rps = Units.radiansPerSecondToRotationsPerMinute(turretSim.getVelocityRadPerSec())/60;
        inputs.targetAngle = Rotation2d.fromRotations(currentTargetAngle);
        inputs.voltage = appliedVoltage;
        inputs.current = turretSim.getCurrentDrawAmps();
    }
    @Override
    public void setVoltage(double voltage){
        isClosedLoop = false;
        appliedVoltage = voltage;
    }

    @Override
    public void setDutyCycle(double percentage){ 
    }

    @Override
    public void setPosition(Rotation2d angle){
        isClosedLoop = true;
        this.currentTargetAngle = angle.getRotations();
    }

    @Override
    public void setPositionWithFF(Rotation2d angle, double feedforward) {
        isClosedLoop = true;
        this.currentTargetAngle = angle.getRotations();
    }
    @Override
    public void resetEncoder(){
    }

    @Override
    public void stop(){
        this.appliedVoltage = 0;
    }


    
}

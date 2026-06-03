// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stzteam.mars.models.containers.IRobotContainer;
import com.stzteam.mars.operator.ControllerOI;
import com.stzteam.mars.test.TestRoutine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Bindings.OperatorBindings;
import frc.robot.Mechanisms.Turret;
import frc.robot.configuration.Manifest;
import frc.tests.EmptyTest;

public class RobotContainer implements IRobotContainer{

  public final ControllerOI driver;
  public final ControllerOI operator;
  
  public final Turret turret;

  public RobotContainer() {
    this.driver = Manifest.ControlsBuilder.buildDriver();
    this.operator = Manifest.ControlsBuilder.buildOperator();

    this.turret = Manifest.buildTurret();

    OperatorBindings.Create(operator, turret);

  }

  @Override
  public void updateNodes() {}

  @Override
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  @Override
  public TestRoutine getTestRoutine() {
    return new EmptyTest();
  }

  
}

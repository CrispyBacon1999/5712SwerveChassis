// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {

  DigitalInput hallEffect = new DigitalInput(0);
  Spark blinkin = new Spark(0);

  /** Creates a new TestSubsystem. */
  public TestSubsystem() {

  }

  @Override
  public void periodic() {
    if (!hallEffect.get()) {
      blinkin.set(-.99);
    } else {
      blinkin.set(.87);
    }
    // This method will be called once per scheduler run
  }
}

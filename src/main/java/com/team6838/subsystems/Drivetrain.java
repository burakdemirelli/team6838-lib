package com.team6838.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team6838.lib.drivers.NavX;
import com.team6838.lib.util.Gearbox;
import com.team6838.lib.util.SRXMagEncoder;
import com.team6838.lib.vision.VisionTarget;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonSRX rearLeft = new WPI_TalonSRX(1);
  private SRXMagEncoder rearLeftEncoder = new SRXMagEncoder(rearLeft);
  private VisionTarget powerPort = new VisionTarget(1.23, 0.5, 30);
  private NavX gyro = new NavX();

  public Drivetrain() {
    rearLeftEncoder.reset();
    rearLeftEncoder.setDistancePerRotation(Units.inchesToMeters(6)*Math.PI*2);
  }

  @Override
  public void periodic() {
    rearLeftEncoder.getPosition();

  }
}

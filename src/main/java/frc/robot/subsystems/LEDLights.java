// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LEDLights extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;

  /** Creates a new LEDLights. */
  public LEDLights() {
    m_led = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(20);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.start();
  }

  public void setRGB(int r, int g, int b) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
   
    m_led.setData(m_ledBuffer);
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  @Override
  public void periodic() {
    if(Robot.getContainer().getColorSensor().getDetectedBallColor().getName().equals("Blue")) {
      setRGB(0, 0, 255);
    } else if(Robot.getContainer().getColorSensor().getDetectedBallColor().getName().equals("Red")) {
      setRGB(255, 0, 0);
    } else {
      setRGB(255, 20, 147);
    }
  }
}

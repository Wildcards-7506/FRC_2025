package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    private AddressableLED ledString;
    private AddressableLEDBuffer ledBuffer;
    private int bufferSize;

    // Store what the last hue of the first pixel is
    private int m_rainbowFirstPixelHue;

    public LED(int PWMPort, int BufferSize){
        this.bufferSize = BufferSize;
        ledString = new AddressableLED(PWMPort);
        ledBuffer = new AddressableLEDBuffer(BufferSize);

        ledString.setLength(BufferSize);
        ledString.setData(ledBuffer);
        ledString.start();
    }

    public void update() {
      ledString.setData(ledBuffer);
    }

    public void rainbow() {
        // For every pixel
        for (var i = 0; i < bufferSize; i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / bufferSize)) % 180;
          // Set the value
          ledBuffer.setHSV(i, hue, 255, 255);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        update();
    }

    public void allianceFlow() {
      //int random = randomInt.nextInt(45) + (red ? 165 : 90);
      // For every pixel
      var alliance = DriverStation.getAlliance();
      boolean red;
      if (alliance.isPresent()){
        red = alliance.get() == DriverStation.Alliance.Red;
      } else{
        red = false;
      }
        // For every pixel
        for (var i = 0; i < bufferSize; i++) {
          final var hue = ((red ? 165 : 90) + (m_rainbowFirstPixelHue + (i * 45 / bufferSize)) % 45)%180;
          // Set the value
          ledBuffer.setHSV(i, hue, 255, 255);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 1;
        // Check bounds
        m_rainbowFirstPixelHue %= 45;
        update();
  }

    public void solid(int hue, int sat, int val) {
        // For every pixel
        for (var i = 0; i < bufferSize; i++) {
          // Set the value
          ledBuffer.setHSV(i, hue, sat, val);
        }
        update();
    }

    public void solidBlink(int hue, int sat, int val) {
        // For every pixel
        for (var i = 0; i < bufferSize; i++) {
          // Set the value
          ledBuffer.setHSV(i, hue, sat, val * (Timer.getFPGATimestamp() % 0.5 < 0.25 ? 1 : 0));
        }
        update();
    }

    public void solidSection(int start, int end, int hue){
      // For every pixel
      for (var i = start; i < end; i++) {
        // Set the value
        ledBuffer.setHSV(i, hue, 255, 255);
      }
      update();
    }
}
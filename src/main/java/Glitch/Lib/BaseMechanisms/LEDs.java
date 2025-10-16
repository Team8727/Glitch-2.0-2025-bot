// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Glitch.Lib.BaseMechanisms;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LEDs extends SubsystemBase {
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  private AddressableLEDBuffer fakeBuffer;
  private final LEDPattern defaultPattern;

  private final double infiniteDurationSeconds = -1.0;
  private double durationSeconds = infiniteDurationSeconds;
  private final double deltaTimeSeconds = 0.02;
  public double elapsedSeconds = 0.0;

  private class Section {
    private final AddressableLEDBufferView bufferView;
    private LEDPattern pattern = defaultPattern;
    private double durationSeconds = LEDs.this.durationSeconds;
    private double elapsedSeconds = LEDs.this.elapsedSeconds;

    public Section(int startIndex, int endIndex) {
      bufferView = ledBuffer.createView(startIndex, endIndex);
    }
   
    /**
     * @param pattern The pattern to set.
     * @param durationSeconds The duration in seconds to set the pattern for.
     */
    public void setPattern(LEDPattern pattern, double durationSeconds) {
      this.pattern = pattern;
      this.durationSeconds = durationSeconds;
      this.elapsedSeconds = 0.0;
    }

    /**
     * Sets the pattern for an infinite duration.
     * @param pattern The pattern to set.
     */
    public void setPattern(LEDPattern pattern) {
      setPattern(pattern, infiniteDurationSeconds);
    }

    /**
     * Ticks down the time until a pattern ends.
     */
    public void update() {
      if(durationSeconds != infiniteDurationSeconds) {
        elapsedSeconds += deltaTimeSeconds;

        if (elapsedSeconds >= durationSeconds) {
          pattern = defaultPattern;
          durationSeconds = infiniteDurationSeconds;
          elapsedSeconds = 0.0;
        }
      }
    }

    /**
     * Ticks down the time until a pattern ends for a section. (put in periodic)
     * @param logic The logic to apply to the LEDs after a pattern ends. Can be "fire" or "noise".
     * @param subPattern The pattern that the LED logic plays over.
     */
    public void update(String logic, LEDPattern subPattern) {
      if(durationSeconds != infiniteDurationSeconds) {
        elapsedSeconds += deltaTimeSeconds;

        if (elapsedSeconds >= durationSeconds) {

          if (logic.equals("fire")) {
            fireAnimation(subPattern, this.bufferView);
          } else if (logic.equals("noise")) {
            activateRandomNoise(subPattern);
          } else {
            System.out.println("This is not an applicable update logic for LEDs");
          }

          durationSeconds = infiniteDurationSeconds;
          elapsedSeconds = 0.0;
        }
      }
    }
    /**
     * Returns the buffer view of the section.
     */
    public AddressableLEDBufferView getBufferView() {
      return this.bufferView;
    }

    /**
     * Returns the length of the buffer view.
     */
    public int getLength() {
      return this.bufferView.getLength();
    }
  }

  /** Creates a new LEDs. 
   * @param ledPort The PWM port number for the LED strip.
   * @param ledLength The number of LEDs in the strip. (make sure that you know if it is individually addressable or not)
   * @param defaultPattern The default pattern to set the LEDs to when other patterns expire.
  */
  public LEDs(int ledPort, int ledLength, LEDPattern defaultPattern) {
    ledStrip = new AddressableLED(ledPort);
    ledBuffer = new AddressableLEDBuffer(ledLength);
    this.defaultPattern = defaultPattern;

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    }

  /**
   * Uses some complicated logic to create a fire animation.
   * @param pattern The pattern that the fire overlays.
   */
  public void fireAnimation (LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
    for (int i = 0; i < ledBuffer.getLength(); i ++) {
      if ((1.5 * (Math.sin(Math.random())) + (i/36.0)) > 1.3) {
        ledBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  /**
   * Uses some complicated logic to create a fire animation.
   * This version is special and only exists to prevent weird update issues. Please don't use it outside of this class.
   * @param pattern The pattern that the fire overlays.
   * @param bufferView The buffer view that the animation will play on.
   */
  public void fireAnimation (LEDPattern pattern, AddressableLEDBufferView bufferView) {
    pattern.applyTo(bufferView);
    for (int i = 0; i < bufferView.getLength(); i ++) {
      if ((1.5 * (Math.sin(Math.random())) + (i/36.0)) > 1.3) {
        bufferView.setRGB(i, 0, 0, 0);
      }
    }
  }

  /**
   * Uses some complicated logic to create a fire animation.
   * @param pattern The pattern that the fire overlays.
   * @param section The section that the animation will play on.
   */
  public void fireAnimation (LEDPattern pattern, Section section) {
      pattern.applyTo(section.getBufferView());
      
      for (int i = 0; i < section.getLength(); i++) {
        if ((1.5 * (Math.sin(Math.random())) + (i / 14.0)) > 1.3) {
          section.getBufferView().setRGB(i, 0, 0, 0);
        }
      }
    }
  

  /**
  * This activates a random noise function separately so that we can run the animation without having to deal with all the logic.
  * May be obsolete.
  * @param pattern The pattern that the random noise overlays.
  */ 
  public void activateRandomNoise(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
    if (fakeBuffer.getLength() != ledBuffer.getLength()) {
      fakeBuffer = new AddressableLEDBuffer(ledBuffer.getLength());
    }
    randomNoiseAnimation(pattern);
  }

  /**
  * This activates a random noise function separately so that we can run the animation without having to deal with all the logic.
  * May be obsolete.
  * @param pattern The pattern that the random noise overlays.
  */ 
  public void activateRandomNoise(LEDPattern pattern, AddressableLEDBufferView bufferView) {
    pattern.applyTo(bufferView);
    if (fakeBuffer.getLength() != bufferView.getLength()) {
      fakeBuffer = new AddressableLEDBuffer(bufferView.getLength());
    }
    randomNoiseAnimation(pattern, bufferView);
  }

  /** 
  * This function creates a fun random noise overlay that took way too long to make.
  * It, like the fire animation, uses LEDPatterns so you could do some cool stuff with it.
  *
  * This wasn't originally intended to work with multiple buffer views at once, so be warned.
  * @param pattern The pattern that the random noise overlays.
  * @param bufferView The buffer view that the animation will play on.
  */
  private void randomNoiseAnimation(LEDPattern pattern) {
    int ledsOn = 0;
    pattern.applyTo(fakeBuffer);
    for (int i = 0; i < ledBuffer.getLength(); i ++) {
      if(!(ledBuffer.getRed(i) == 0 && ledBuffer.getGreen(i) == 0 && ledBuffer.getBlue(i) == 0)) {
        ledsOn += 1;
      }
    }
    for (int i = 0; i < ledBuffer.getLength(); i ++) {
      if ((ledsOn == 0) || Math.random() > 0.5) {
        ledBuffer.setRGB(i, fakeBuffer.getRed(i), fakeBuffer.getGreen(i), fakeBuffer.getBlue(i));
        ledsOn += 1;
      }
      if ((Math.random() * ledBuffer.getLength()) < (ledsOn) * 0.5) {
        ledBuffer.setRGB(i, 0, 0, 0);
        ledsOn -= 1;
      }
    }
  }

  /** 
  * This function creates a fun random noise overlay that took way too long to make.
  * It, like the fire animation, uses LEDPatterns so you could do some cool stuff with it.
  *
  * This wasn't originally intended to work with multiple buffer views at once, so be warned.
  * @param pattern The pattern that the random noise overlays.
  * @param bufferView The buffer view that the animation will play on.
  */
  private void randomNoiseAnimation(LEDPattern pattern, AddressableLEDBufferView bufferView) {
    int ledsOn = 0;
    pattern.applyTo(fakeBuffer);
    for (int i = 0; i < bufferView.getLength(); i ++) {
      if(!(bufferView.getRed(i) == 0 && bufferView.getGreen(i) == 0 && bufferView.getBlue(i) == 0)) {
        ledsOn += 1;
      }
    }
    for (int i = 0; i < bufferView.getLength(); i ++) {
      if ((ledsOn == 0) || Math.random() > 0.5) {
        bufferView.setRGB(i, fakeBuffer.getRed(i), fakeBuffer.getGreen(i), fakeBuffer.getBlue(i));
        ledsOn += 1;
      }
      if ((Math.random() * bufferView.getLength()) < (ledsOn) * 0.5) {
        bufferView.setRGB(i, 0, 0, 0);
        ledsOn -= 1;
      }
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ledStrip.setData(ledBuffer);
  }
}

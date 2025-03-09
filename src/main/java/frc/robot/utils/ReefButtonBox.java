package frc.robot.utils;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Handle input from Xbox controllers connected to the Driver Station.
 *
 * <p>This class handles Xbox input that comes from the Driver Station. Each time a value is
 * requested the most recent value is returned. There is a single class instance for each controller
 * and the mapping of ports to hardware buttons depends on the code in the Driver Station.
 *
 * <p>Only first party controllers from Microsoft are guaranteed to have the correct mapping, and
 * only through the official NI DS. Sim is not guaranteed to have the same mapping, as well as any
 * 3rd party controllers.
 */
public class ReefButtonBox extends GenericHID implements Sendable {
  /** Represents a digital button on a XboxController. */
  public enum Button {
    /** Reef BUttons A - L */
    kA(1),
    kB(2),
    kC(3),
    kD(4),
    kE(5),
    kF(6),
    kG(7),
    kH(8),
    kI(9),
    kJ(10),
    kK(11),
    kL(12),
    kL2(13),
    kL3(14),
    kL4(15),
    kClimbIn(16),
    kClimbOut(17),
    kClearHigh(18),
    kClearLow(19),
    kElevatorDown(20);
  



    /** Button value. */
    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and appending `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into (0-5).
   */
  public ReefButtonBox(final int port) {
    super(port);
    HAL.report(tResourceType.kResourceType_XboxController, port + 1);
  }



  /**
   * Read the value of the A button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getAButton() {
    return getRawButton(Button.kA.value);
  }

  /**
   * Whether the A button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getAButtonPressed() {
    return getRawButtonPressed(Button.kA.value);
  }

  /**
   * Whether the A button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getAButtonReleased() {
    return getRawButtonReleased(Button.kA.value);
  }

  /**
   * Constructs an event instance around the A button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent a(EventLoop loop) {
    return button(Button.kA.value, loop);
  }

  /**
   * Read the value of the B button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getBButton() {
    return getRawButton(Button.kB.value);
  }

  /**
   * Whether the B button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getBButtonPressed() {
    return getRawButtonPressed(Button.kB.value);
  }

  /**
   * Whether the B button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getBButtonReleased() {
    return getRawButtonReleased(Button.kB.value);
  }

  /**
   * Constructs an event instance around the B button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the B button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent b(EventLoop loop) {
    return button(Button.kB.value, loop);
  }

/**
 * Read the value of the C button on the controller.
 *
 * @return The state of the button.
 */
public boolean getCButton() {
    return getRawButton(Button.kC.value);
}

/**
 * Whether the C button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
public boolean getCButtonPressed() {
    return getRawButtonPressed(Button.kC.value);
}

/**
 * Whether the C button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
public boolean getCButtonReleased() {
    return getRawButtonReleased(Button.kC.value);
}

/**
 * Constructs an event instance around the C button's digital signal.
 *
 * @param loop the event loop instance to attach the event to.
 * @return an event instance representing the C button's digital signal
 *     attached to the given loop.
 */
public BooleanEvent c(EventLoop loop) {
    return button(Button.kC.value, loop);
}

// Repeat the same pattern for buttons D to L

public boolean getDButton() {
    return getRawButton(Button.kD.value);
}

public boolean getDButtonPressed() {
    return getRawButtonPressed(Button.kD.value);
}

public boolean getDButtonReleased() {
    return getRawButtonReleased(Button.kD.value);
}

public BooleanEvent d(EventLoop loop) {
    return button(Button.kD.value, loop);
}

public boolean getEButton() {
    return getRawButton(Button.kE.value);
}

public boolean getEButtonPressed() {
    return getRawButtonPressed(Button.kE.value);
}

public boolean getEButtonReleased() {
    return getRawButtonReleased(Button.kE.value);
}

public BooleanEvent e(EventLoop loop) {
    return button(Button.kE.value, loop);
}

public boolean getFButton() {
    return getRawButton(Button.kF.value);
}

public boolean getFButtonPressed() {
    return getRawButtonPressed(Button.kF.value);
}

public boolean getFButtonReleased() {
    return getRawButtonReleased(Button.kF.value);
}

public BooleanEvent f(EventLoop loop) {
    return button(Button.kF.value, loop);
}

public boolean getGButton() {
    return getRawButton(Button.kG.value);
}

public boolean getGButtonPressed() {
    return getRawButtonPressed(Button.kG.value);
}

public boolean getGButtonReleased() {
    return getRawButtonReleased(Button.kG.value);
}

public BooleanEvent g(EventLoop loop) {
    return button(Button.kG.value, loop);
}

public boolean getHButton() {
    return getRawButton(Button.kH.value);
}

public boolean getHButtonPressed() {
    return getRawButtonPressed(Button.kH.value);
}

public boolean getHButtonReleased() {
    return getRawButtonReleased(Button.kH.value);
}

public BooleanEvent h(EventLoop loop) {
    return button(Button.kH.value, loop);
}

public boolean getIButton() {
    return getRawButton(Button.kI.value);
}

public boolean getIButtonPressed() {
    return getRawButtonPressed(Button.kI.value);
}

public boolean getIButtonReleased() {
    return getRawButtonReleased(Button.kI.value);
}

public BooleanEvent i(EventLoop loop) {
    return button(Button.kI.value, loop);
}

public boolean getJButton() {
    return getRawButton(Button.kJ.value);
}

public boolean getJButtonPressed() {
    return getRawButtonPressed(Button.kJ.value);
}

public boolean getJButtonReleased() {
    return getRawButtonReleased(Button.kJ.value);
}

public BooleanEvent j(EventLoop loop) {
    return button(Button.kJ.value, loop);
}

public boolean getKButton() {
    return getRawButton(Button.kK.value);
}

public boolean getKButtonPressed() {
    return getRawButtonPressed(Button.kK.value);
}

public boolean getKButtonReleased() {
    return getRawButtonReleased(Button.kK.value);
}

public BooleanEvent k(EventLoop loop) {
    return button(Button.kK.value, loop);
}

public boolean getLButton() {
    return getRawButton(Button.kL.value);
}

public boolean getLButtonPressed() {
    return getRawButtonPressed(Button.kL.value);
}

public boolean getLButtonReleased() {
    return getRawButtonReleased(Button.kL.value);
}

public BooleanEvent l(EventLoop loop) {
    return button(Button.kL.value, loop);
}
public boolean getL2Button() {
    return getRawButton(Button.kL2.value);
}

public boolean getL2ButtonPressed() {
    return getRawButtonPressed(Button.kL2.value);
}

public boolean getL2ButtonReleased() {
    return getRawButtonReleased(Button.kL2.value);
}

public BooleanEvent l2(EventLoop loop) {
    return button(Button.kL2.value, loop);
}

public boolean getL3Button() {
    return getRawButton(Button.kL3.value);
}

public boolean getL3ButtonPressed() {
    return getRawButtonPressed(Button.kL3.value);
}

public boolean getL3ButtonReleased() {
    return getRawButtonReleased(Button.kL3.value);
}

public BooleanEvent l3(EventLoop loop) {
    return button(Button.kL3.value, loop);
}

public boolean getL4Button() {
    return getRawButton(Button.kL4.value);
}

public boolean getL4ButtonPressed() {
    return getRawButtonPressed(Button.kL4.value);
}

public boolean getL4ButtonReleased() {
    return getRawButtonReleased(Button.kL4.value);
}

public BooleanEvent l4(EventLoop loop) {
    return button(Button.kL4.value, loop);
}

public boolean getClimbInButton() {
    return getRawButton(Button.kClimbIn.value);
}

public boolean getClimbInButtonPressed() {
    return getRawButtonPressed(Button.kClimbIn.value);
}

public boolean getClimbInButtonReleased() {
    return getRawButtonReleased(Button.kClimbIn.value);
}

public BooleanEvent climbIn(EventLoop loop) {
    return button(Button.kClimbIn.value, loop);
}

public boolean getClimbOutButton() {
    return getRawButton(Button.kClimbOut.value);
}

public boolean getClimbOutButtonPressed() {
    return getRawButtonPressed(Button.kClimbOut.value);
}

public boolean getClimbOutButtonReleased() {
    return getRawButtonReleased(Button.kClimbOut.value);
}

public BooleanEvent climbOut(EventLoop loop) {
    return button(Button.kClimbOut.value, loop);
}

public boolean getClearLowButton() {
    return getRawButton(Button.kClearLow.value);
}

public boolean getClearLowButtonPressed() {
    return getRawButtonPressed(Button.kClearLow.value);
}

public boolean getClearLowButtonReleased() {
    return getRawButtonReleased(Button.kClearLow.value);
}

public BooleanEvent clearLow(EventLoop loop) {
    return button(Button.kClearLow.value, loop);
}

public boolean getClearHighButton() {
    return getRawButton(Button.kClearHigh.value);
}

public boolean getClearHighButtonPressed() {
    return getRawButtonPressed(Button.kClearHigh.value);
}

public boolean getClearHighButtonReleased() {
    return getRawButtonReleased(Button.kClearHigh.value);
}

public BooleanEvent clearHigh(EventLoop loop) {
    return button(Button.kClearHigh.value, loop);
}

public boolean getElevatorDownButton() {
    return getRawButton(Button.kElevatorDown.value);
}

public boolean getElevatorDownButtonPressed() {
    return getRawButtonPressed(Button.kElevatorDown.value);
}

public boolean getElevatorDownButtonReleased() {
    return getRawButtonReleased(Button.kElevatorDown.value);
}

public BooleanEvent elevatorDown(EventLoop loop) {
    return button(Button.kElevatorDown.value, loop);
}

public boolean getAnyReefPositionPressed(){
    //check getButton for each enum value in  A- L
    return getAButton() || getBButton() || getCButton() || getDButton() || getEButton() || getFButton() || getGButton() || getHButton() || getIButton() || getJButton() || getKButton() || getLButton();
}

public Button getWhichReefButtonPressed(){
    //check getButton for each enum value in  A- L
    if(getAButton()) return Button.kA;
    if(getBButton()) return Button.kB;
    if(getCButton()) return Button.kC;
    if(getDButton()) return Button.kD;
    if(getEButton()) return Button.kE;
    if(getFButton()) return Button.kF;
    if(getGButton()) return Button.kG;
    if(getHButton()) return Button.kH;
    if(getIButton()) return Button.kI;
    if(getJButton()) return Button.kJ;
    if(getKButton()) return Button.kK;
    if(getLButton()) return Button.kL;
    return null;
}

public Button getWhichHeightButtonPressed(){
    //check getButton for each enum value in  A- L
    if(getL2Button()) return Button.kL2;
    if(getL3Button()) return Button.kL3;
    if(getL4Button()) return Button.kL4;
    return null;
}

public boolean getAnyHeightPositionPressed(){
    //check getButton for each enum value in  A- L
    return getL2Button() || getL3Button() || getL4Button();
}

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HID");
    builder.publishConstString("ControllerType", "Xbox");

    builder.addBooleanProperty("A", this::getAButton, null);
    builder.addBooleanProperty("B", this::getBButton, null);
    builder.addBooleanProperty("C", this::getBButton, null);
    builder.addBooleanProperty("D", this::getDButton, null);
    builder.addBooleanProperty("E", this::getEButton, null);
    builder.addBooleanProperty("F", this::getFButton, null);
    builder.addBooleanProperty("G", this::getGButton, null);
    builder.addBooleanProperty("H", this::getHButton, null);
    builder.addBooleanProperty("I", this::getIButton, null);
    builder.addBooleanProperty("J", this::getJButton, null);
    builder.addBooleanProperty("K", this::getKButton, null);
    builder.addBooleanProperty("L", this::getLButton, null);
    builder.addBooleanProperty("L2", this::getL2Button, null);
    builder.addBooleanProperty("L3", this::getL3Button, null);
    builder.addBooleanProperty("L4", this::getL4Button, null);
    builder.addBooleanProperty("ClimbIn", this::getClimbInButton, null);
    builder.addBooleanProperty("ClimbOut", this::getClimbOutButton, null);
    builder.addBooleanProperty("ClearLow", this::getClearLowButton, null);
    builder.addBooleanProperty("ClearHigh", this::getClearHighButton, null);
    builder.addBooleanProperty("ElevatorDown", this::getElevatorDownButton, null);
    

   
  }
}


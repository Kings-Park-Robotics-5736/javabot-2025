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
    
    k2(0),
    k4(1),
    k6(2),
    k8(3),
    k10(4),
    k12(5),
    kClear(6),
    kRL4(7),
    kRL3(8),
    kRL2(9),
    kLL4(10),
    kLL3(11),
    kLL2(12),
    kClimbOut(13),
    kClimbIn(14),
    

    kElevatorDown(15);
  



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
  public boolean getK12Button() {
    return getRawButton(Button.k12.value);
  }

  /**
   * Whether the A button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getK12ButtonPressed() {
    return getRawButtonPressed(Button.k12.value);
  }

  /**
   * Whether the A button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getK12ButtonReleased() {
    return getRawButtonReleased(Button.k12.value);
  }

  /**
   * Constructs an event instance around the A button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent k12(EventLoop loop) {
    return button(Button.k12.value, loop);
  }

  // ...existing code...

/**
   * Read the value of the k2 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getK2Button() {
    return getRawButton(Button.k2.value);
  }

  /**
   * Whether the k2 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getK2ButtonPressed() {
    return getRawButtonPressed(Button.k2.value);
  }

  /**
   * Whether the k2 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getK2ButtonReleased() {
    return getRawButtonReleased(Button.k2.value);
  }

  /**
   * Constructs an event instance around the k2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the k2 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent k2(EventLoop loop) {
    return button(Button.k2.value, loop);
  }

/**
   * Read the value of the k4 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getK4Button() {
    return getRawButton(Button.k4.value);
  }

  /**
   * Whether the k4 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getK4ButtonPressed() {
    return getRawButtonPressed(Button.k4.value);
  }

  /**
   * Whether the k4 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getK4ButtonReleased() {
    return getRawButtonReleased(Button.k4.value);
  }

  /**
   * Constructs an event instance around the k4 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the k4 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent k4(EventLoop loop) {
    return button(Button.k4.value, loop);
  }

/**
   * Read the value of the k6 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getK6Button() {
    return getRawButton(Button.k6.value);
  }

  /**
   * Whether the k6 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getK6ButtonPressed() {
    return getRawButtonPressed(Button.k6.value);
  }

  /**
   * Whether the k6 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getK6ButtonReleased() {
    return getRawButtonReleased(Button.k6.value);
  }

  /**
   * Constructs an event instance around the k6 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the k6 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent k6(EventLoop loop) {
    return button(Button.k6.value, loop);
  }

/**
   * Read the value of the k8 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getK8Button() {
    return getRawButton(Button.k8.value);
  }

  /**
   * Whether the k8 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getK8ButtonPressed() {
    return getRawButtonPressed(Button.k8.value);
  }

  /**
   * Whether the k8 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getK8ButtonReleased() {
    return getRawButtonReleased(Button.k8.value);
  }

  /**
   * Constructs an event instance around the k8 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the k8 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent k8(EventLoop loop) {
    return button(Button.k8.value, loop);
  }

/**
   * Read the value of the k10 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getK10Button() {
    return getRawButton(Button.k10.value);
  }

  /**
   * Whether the k10 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getK10ButtonPressed() {
    return getRawButtonPressed(Button.k10.value);
  }

  /**
   * Whether the k10 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getK10ButtonReleased() {
    return getRawButtonReleased(Button.k10.value);
  }

  /**
   * Constructs an event instance around the k10 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the k10 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent k10(EventLoop loop) {
    return button(Button.k10.value, loop);
  }

// ...existing code...

// ...existing code...

/**
   * Read the value of the kClear button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getKClearButton() {
    return getRawButton(Button.kClear.value);
  }

  /**
   * Whether the kClear button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getKClearButtonPressed() {
    return getRawButtonPressed(Button.kClear.value);
  }

  /**
   * Whether the kClear button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getKClearButtonReleased() {
    return getRawButtonReleased(Button.kClear.value);
  }

  /**
   * Constructs an event instance around the kClear button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the kClear button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent kClear(EventLoop loop) {
    return button(Button.kClear.value, loop);
  }

/**
   * Read the value of the kLL4 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getKLL4Button() {
    return getRawButton(Button.kLL4.value);
  }

  /**
   * Whether the kLL4 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getKLL4ButtonPressed() {
    return getRawButtonPressed(Button.kLL4.value);
  }

  /**
   * Whether the kLL4 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getKLL4ButtonReleased() {
    return getRawButtonReleased(Button.kLL4.value);
  }

  /**
   * Constructs an event instance around the kLL4 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the kLL4 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent kLL4(EventLoop loop) {
    return button(Button.kLL4.value, loop);
  }

/**
   * Read the value of the kLL3 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getKLL3Button() {
    return getRawButton(Button.kLL3.value);
  }

  /**
   * Whether the kLL3 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getKLL3ButtonPressed() {
    return getRawButtonPressed(Button.kLL3.value);
  }

  /**
   * Whether the kLL3 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getKLL3ButtonReleased() {
    return getRawButtonReleased(Button.kLL3.value);
  }

  /**
   * Constructs an event instance around the kLL3 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the kLL3 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent kLL3(EventLoop loop) {
    return button(Button.kLL3.value, loop);
  }

/**
   * Read the value of the kLL2 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getKLL2Button() {
    return getRawButton(Button.kLL2.value);
  }

  /**
   * Whether the kLL2 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getKLL2ButtonPressed() {
    return getRawButtonPressed(Button.kLL2.value);
  }

  /**
   * Whether the kLL2 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getKLL2ButtonReleased() {
    return getRawButtonReleased(Button.kLL2.value);
  }

  /**
   * Constructs an event instance around the kLL2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the kLL2 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent kLL2(EventLoop loop) {
    return button(Button.kLL2.value, loop);
  }

/**
   * Read the value of the kRL4 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getKRL4Button() {
    return getRawButton(Button.kRL4.value);
  }

  /**
   * Whether the kRL4 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getKRL4ButtonPressed() {
    return getRawButtonPressed(Button.kRL4.value);
  }

  /**
   * Whether the kRL4 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getKRL4ButtonReleased() {
    return getRawButtonReleased(Button.kRL4.value);
  }

  /**
   * Constructs an event instance around the kRL4 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the kRL4 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent kRL4(EventLoop loop) {
    return button(Button.kRL4.value, loop);
  }

/**
   * Read the value of the kRL3 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getKRL3Button() {
    return getRawButton(Button.kRL3.value);
  }

  /**
   * Whether the kRL3 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getKRL3ButtonPressed() {
    return getRawButtonPressed(Button.kRL3.value);
  }

  /**
   * Whether the kRL3 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getKRL3ButtonReleased() {
    return getRawButtonReleased(Button.kRL3.value);
  }

  /**
   * Constructs an event instance around the kRL3 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the kRL3 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent kRL3(EventLoop loop) {
    return button(Button.kRL3.value, loop);
  }

/**
   * Read the value of the kRL2 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getKRL2Button() {
    return getRawButton(Button.kRL2.value);
  }

  /**
   * Whether the kRL2 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getKRL2ButtonPressed() {
    return getRawButtonPressed(Button.kRL2.value);
  }

  /**
   * Whether the kRL2 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getKRL2ButtonReleased() {
    return getRawButtonReleased(Button.kRL2.value);
  }

  /**
   * Constructs an event instance around the kRL2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the kRL2 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent kRL2(EventLoop loop) {
    return button(Button.kRL2.value, loop);
  }

// ...existing code...

// ...existing code...

/**
   * Read the value of the kClimbIn button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getKClimbInButton() {
    return getRawButton(Button.kClimbIn.value);
  }

  /**
   * Whether the kClimbIn button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getKClimbInButtonPressed() {
    return getRawButtonPressed(Button.kClimbIn.value);
  }

  /**
   * Whether the kClimbIn button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getKClimbInButtonReleased() {
    return getRawButtonReleased(Button.kClimbIn.value);
  }

  /**
   * Constructs an event instance around the kClimbIn button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the kClimbIn button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent kClimbIn(EventLoop loop) {
    return button(Button.kClimbIn.value, loop);
  }

/**
   * Read the value of the kClimbOut button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getKClimbOutButton() {
    return getRawButton(Button.kClimbOut.value);
  }

  /**
   * Whether the kClimbOut button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getKClimbOutButtonPressed() {
    return getRawButtonPressed(Button.kClimbOut.value);
  }

  /**
   * Whether the kClimbOut button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getKClimbOutButtonReleased() {
    return getRawButtonReleased(Button.kClimbOut.value);
  }

  /**
   * Constructs an event instance around the kClimbOut button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the kClimbOut button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent kClimbOut(EventLoop loop) {
    return button(Button.kClimbOut.value, loop);
  }

/**
   * Read the value of the kElevatorDown button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getKElevatorDownButton() {
    return getRawButton(Button.kElevatorDown.value);
  }

  /**
   * Whether the kElevatorDown button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getKElevatorDownButtonPressed() {
    return getRawButtonPressed(Button.kElevatorDown.value);
  }

  /**
   * Whether the kElevatorDown button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getKElevatorDownButtonReleased() {
    return getRawButtonReleased(Button.kElevatorDown.value);
  }

  /**
   * Constructs an event instance around the kElevatorDown button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the kElevatorDown button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent kElevatorDown(EventLoop loop) {
    return button(Button.kElevatorDown.value, loop);
  }

// ...existing code...


  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HID");
    builder.publishConstString("ControllerType", "Xbox");

    builder.addBooleanProperty("AK12", this::getK12Button, null);
    builder.addBooleanProperty("AK2", this::getK2Button, null);
    builder.addBooleanProperty("AK4", this::getK4Button, null);
    builder.addBooleanProperty("AK6", this::getK6Button, null);
    builder.addBooleanProperty("AK8", this::getK8Button, null);
    builder.addBooleanProperty("AK10", this::getK10Button, null);
    builder.addBooleanProperty("AKClear", this::getKClearButton, null);
    builder.addBooleanProperty("AKLL4", this::getKLL4Button, null);
    builder.addBooleanProperty("AKLL3", this::getKLL3Button, null);
    builder.addBooleanProperty("AKLL2", this::getKLL2Button, null);
    builder.addBooleanProperty("AKRL4", this::getKRL4Button, null);
    builder.addBooleanProperty("AKRL3", this::getKRL3Button, null);
    builder.addBooleanProperty("AKRL2", this::getKRL2Button, null);
    builder.addBooleanProperty("AKClimbIn", this::getKClimbInButton, null);
    builder.addBooleanProperty("AKClimbOut", this::getKClimbOutButton, null);
    builder.addBooleanProperty("AKElevatorDown", this::getKElevatorDownButton, null);

  
    

   
  }
}

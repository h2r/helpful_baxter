package baxter_core_msgs;

public interface DigitalIOState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/DigitalIOState";
  static final java.lang.String _DEFINITION = "int8 state\nbool isInputOnly\n\nint8 OFF = 0\nint8 ON  = 1\nint8 PRESSED = 1\nint8 UNPRESSED = 0";
  static final byte OFF = 0;
  static final byte ON = 1;
  static final byte PRESSED = 1;
  static final byte UNPRESSED = 0;
  byte getState();
  void setState(byte value);
  boolean getIsInputOnly();
  void setIsInputOnly(boolean value);
}

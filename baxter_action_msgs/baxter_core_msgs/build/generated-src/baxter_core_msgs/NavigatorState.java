package baxter_core_msgs;

public interface NavigatorState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/NavigatorState";
  static final java.lang.String _DEFINITION = "# OK button\nbool ok\n# Cancel button\nbool cancel\n# Show button\nbool show\n# Wheel button\nuint8   wheel\n\n# true if the inner light is on, false if not\nbool innerLight\n\n# true if the outer light is on, false if not\nbool outerLight\n";
  boolean getOk();
  void setOk(boolean value);
  boolean getCancel();
  void setCancel(boolean value);
  boolean getShow();
  void setShow(boolean value);
  byte getWheel();
  void setWheel(byte value);
  boolean getInnerLight();
  void setInnerLight(boolean value);
  boolean getOuterLight();
  void setOuterLight(boolean value);
}

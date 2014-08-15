package baxter_core_msgs;

public interface ITBState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/ITBState";
  static final java.lang.String _DEFINITION = "bool[4] buttons\nbool    up\nbool    down\nbool    left\nbool    right\nuint8   wheel\n\n# true if the inner light is on, false if not\nbool innerLight\n\n# true if the outer light is on, false if not\nbool outerLight\n";
  boolean[] getButtons();
  void setButtons(boolean[] value);
  boolean getUp();
  void setUp(boolean value);
  boolean getDown();
  void setDown(boolean value);
  boolean getLeft();
  void setLeft(boolean value);
  boolean getRight();
  void setRight(boolean value);
  byte getWheel();
  void setWheel(byte value);
  boolean getInnerLight();
  void setInnerLight(boolean value);
  boolean getOuterLight();
  void setOuterLight(boolean value);
}

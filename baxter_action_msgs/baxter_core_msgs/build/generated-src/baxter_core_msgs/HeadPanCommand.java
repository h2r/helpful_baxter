package baxter_core_msgs;

public interface HeadPanCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/HeadPanCommand";
  static final java.lang.String _DEFINITION = "#Header header\nfloat32 target # radians for target, 0 str\nint32 speed # between 0 and 100, 100 = max\n\n";
  float getTarget();
  void setTarget(float value);
  int getSpeed();
  void setSpeed(int value);
}

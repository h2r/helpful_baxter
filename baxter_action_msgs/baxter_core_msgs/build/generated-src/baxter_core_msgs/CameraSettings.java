package baxter_core_msgs;

public interface CameraSettings extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/CameraSettings";
  static final java.lang.String _DEFINITION = "int32           width\nint32           height\nfloat32         fps\nCameraControl[] controls\n";
  int getWidth();
  void setWidth(int value);
  int getHeight();
  void setHeight(int value);
  float getFps();
  void setFps(float value);
  java.util.List<baxter_core_msgs.CameraControl> getControls();
  void setControls(java.util.List<baxter_core_msgs.CameraControl> value);
}

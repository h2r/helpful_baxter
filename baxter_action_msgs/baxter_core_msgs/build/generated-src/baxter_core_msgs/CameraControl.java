package baxter_core_msgs;

public interface CameraControl extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/CameraControl";
  static final java.lang.String _DEFINITION = "int32   id\nint32   value\n\nint32 CAMERA_CONTROL_EXPOSURE=100\nint32 CAMERA_CONTROL_GAIN=101\nint32 CAMERA_CONTROL_WHITE_BALANCE_R=102\nint32 CAMERA_CONTROL_WHITE_BALANCE_G=103\nint32 CAMERA_CONTROL_WHITE_BALANCE_B=104\nint32 CAMERA_CONTROL_WINDOW_X=105\nint32 CAMERA_CONTROL_WINDOW_Y=106\nint32 CAMERA_CONTROL_FLIP=107\nint32 CAMERA_CONTROL_MIRROR=108\nint32 CAMERA_CONTROL_RESOLUTION_HALF=109\n";
  static final int CAMERA_CONTROL_EXPOSURE = 100;
  static final int CAMERA_CONTROL_GAIN = 101;
  static final int CAMERA_CONTROL_WHITE_BALANCE_R = 102;
  static final int CAMERA_CONTROL_WHITE_BALANCE_G = 103;
  static final int CAMERA_CONTROL_WHITE_BALANCE_B = 104;
  static final int CAMERA_CONTROL_WINDOW_X = 105;
  static final int CAMERA_CONTROL_WINDOW_Y = 106;
  static final int CAMERA_CONTROL_FLIP = 107;
  static final int CAMERA_CONTROL_MIRROR = 108;
  static final int CAMERA_CONTROL_RESOLUTION_HALF = 109;
  int getId();
  void setId(int value);
  int getValue();
  void setValue(int value);
}

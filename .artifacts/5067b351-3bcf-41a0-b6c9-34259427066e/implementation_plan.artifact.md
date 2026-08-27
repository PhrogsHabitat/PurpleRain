# PS5 Controller Advanced Features Support

This plan addresses the requirement to utilize PS5 controller features, specifically **Adaptive Triggers**, in an FTC robot. Since the controllers are connected to the Driver Hub, direct USB access from the Robot Controller is not possible. The proposed workaround involves a two-pronged approach:
1.  Providing a robust `PlayStationController` class that implements the DualSense HID protocol.
2.  Implementing a network-based communication strategy to send trigger commands from the Robot Controller to the Driver Hub.

## User Review Required

> [!IMPORTANT]
> To use Adaptive Triggers while the controller is plugged into the **Driver Hub**, a small companion application (or a background service) must be installed on the Driver Hub to act as a proxy. The standard FTC Driver Station app does not currently expose an API for sending raw HID reports to gamepads.

> [!TIP]
> If you are able to plug the controller into the **Control Hub** (on the robot), no additional software is needed, and the provided `PlayStationController` class will work immediately via direct USB.

## Proposed Changes

### Core Logic

#### [MODIFY] [PlayStationController.java](file:///C:/Users/Sean/Desktop/PurpleRain/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Purple/PlayStationController.java)
- Complete the DualSense HID report generation logic for Adaptive Triggers.
- Add support for different trigger modes: Off, Resistance, Weapon, and Vibration.
- Refactor the USB initialization to be more robust.

#### [MODIFY] [Controls.java](file:///C:/Users/Sean/Desktop/PurpleRain/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Purple/Controls.java)
- Add a reference to `PlayStationController`.
- Implement `setTriggerFeedback` using the new HID logic.
- Add methods for setting LED colors and advanced rumble.

### Tools & Utilities

#### [NEW] [DualSenseProxy.java](file:///C:/Users/Sean/Desktop/PurpleRain/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Purple/Utils/DualSenseProxy.java)
- A helper class to handle network communication between the RC and the potential companion app on the Driver Hub.

#### [NEW] [PS5AdaptiveTriggerTest.java](file:///C:/Users/Sean/Desktop/PurpleRain/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Purple/Tools/PS5AdaptiveTriggerTest.java)
- An OpMode to test different adaptive trigger settings (resistance levels, modes) in real-time.

## Verification Plan

### Manual Verification
1.  **Direct Connection Test**: Plug a PS5 controller into the Control Hub USB port and run `PS5AdaptiveTriggerTest`. Verify that triggers respond to setting changes.
2.  **Network Proxy Test**: (If companion app is available) Run the proxy on the Driver Hub and verify that commands from the RC are received and applied.
3.  **Touchpad Verification**: Use `TouchpadTest` to ensure that standard touchpad features still work as expected.

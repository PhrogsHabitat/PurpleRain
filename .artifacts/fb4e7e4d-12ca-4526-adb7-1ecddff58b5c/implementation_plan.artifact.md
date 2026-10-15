# Fix PS5 Adaptive Trigger Support

The `PS5AdaptiveTriggerTest.java` class is currently broken due to missing methods and enums in `Controls` and `PlayStationController`. This plan aims to restore the functionality by updating these classes to match the expected API.

## Proposed Changes

### [PlayStationController.java](file:///C:/Users/Ballistabots/Desktop/PurpleRain/PurpleRain/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Purple/PlayStationController.java)
- [NEW] Add `TriggerMode` enum: `OFF`, `FEEDBACK`, `WEAPON`, `VIBRATION`.
- [MODIFY] Update `sendTriggerEffect` to handle different modes more comprehensively if possible, or at least accept the enum.
- [MODIFY] Add more robust report generation for different trigger modes.

### [Controls.java](file:///C:/Users/Ballistabots/Desktop/PurpleRain/PurpleRain/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Purple/Controls.java)
- [MODIFY] Update constructor to accept `id` and `Context` to initialize `PlayStationController`.
- [NEW] Add `private PlayStationController psController`.
- [NEW] Implement `setAdaptiveTrigger` overloads:
    - `setAdaptiveTrigger(boolean right, PlayStationController.TriggerMode mode)`
    - `setAdaptiveTrigger(boolean right, PlayStationController.TriggerMode mode, int startPos, int resistance)`
    - `setAdaptiveTrigger(boolean right, PlayStationController.TriggerMode mode, int startPos, int endPos, int resistance)`
    - `setAdaptiveTrigger(boolean right, PlayStationController.TriggerMode mode, int startPos, int amplitude, int frequency)`

### [PS5AdaptiveTriggerTest.java](file:///C:/Users/Ballistabots/Desktop/PurpleRain/PurpleRain/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Purple/Tools/PS5AdaptiveTriggerTest.java)
- [MODIFY] The code should compile once the dependencies are fixed. I will verify if any other adjustments are needed.

## Verification Plan
### Automated Tests
- Run `gradle build` to ensure all classes compile.
- If possible, run the `PS5AdaptiveTriggerTest` OpMode on a robot with a DualSense controller (manual verification by user).

### Manual Verification
- Verify that the `TeleOp` appears in the driver station.
- User needs to test if the adaptive triggers actually respond to the commands.

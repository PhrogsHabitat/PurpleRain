# PurpleRain Repo Code Style Guide

This document explains the conventions and tools used for writing **PurpleRain’s** Java code for the FTC SDK. Keeping a consistent code style makes the repo easier to maintain and navigate for everyone.

---

# ORGANIZATION

- We ONLY need variables for stored values if their value is used more than once or is intended to change or be tweaked.  
  If this is not the case, use an inline value where it’s needed. This applies to **ALL types of variables**.

- If the purpose of a variable is to clarify a value’s meaning, create it as a **local variable in the area it’s used**, not as a class field.

- **IMPORTANT:**  
  Do NOT make redundant variables that simply reference other variables unless absolutely necessary.  
  The variables section is the most cluttered area of code in almost all cases — keep it clean.

- Public fields first, private fields after.

- Core lifecycle methods ALWAYS come first:
  - `Create()`
  - `Update()`
  - `Destroy()`
  - `runOpMode()` (for LinearOpMode)

- Public utility methods go under lifecycle methods and require simple documentation.
  - Create-like methods first
  - Update-like methods after

- All private, non-utility methods go at the bottom and do not need documentation.
  - Create-like methods first
  - Update-like methods after

- Do NOT create useless wrapper methods.

- Do NOT separate logic into multiple small methods for no reason.  
  If `updateDrive()` and `updateLift()` can reasonably live inside `loop()`, they should.  
  Only extract logic when it improves clarity or reuse.

- Classes that manage their own update loops or represent independent subsystems (e.g., DriveTrain, Lift, Intake) should be placed in their own files.

---

# Whitespace & Indentation

- Use **4 spaces** for indentation (default Android Studio setting).
- No trailing whitespace at the end of lines.
- Add a single blank line between method definitions.
- Add a blank line before `return` statements when appropriate for clarity.
- Always use braces `{}` for control blocks, even for single-line statements.

```java
if (isReady)
{
    startMotor();
}
```

---

# Variable & Method Names

## camelCase

Use **camelCase** for:
- Variables
- Private methods
- Parameters

```java
int currentState;
double liftHeight;

void resetEncoder()
{

}
```

## PascalCase

Use **PascalCase** for:
- Class names
- Public methods
- Enums

```java
public class DriveTrain
{

}

public void SetTargetPosition(int position)
{

}
```

Descriptive names are preferred over short ones — clarity over brevity.

---

# Naming: "Purple" Prefix for Core Classes Only

Only **core system classes** should use the `Purple` prefix.

Examples:
- `PurpleHardware`
- `PurpleDrive`
- `PurpleConfig`

Do NOT use the `Purple` prefix for:
- Derived classes
- Subsystems
- Feature-specific utilities

Example:  
If `PurpleHardware` is your base hardware map manager, a subsystem extending it should NOT be called `PurpleLift`.

Reserve the `Purple` prefix for foundational/core systems only.

This keeps the architecture clear and organized.

---

# Code Comments

Use **Javadoc comments** for all public utility methods, properties, and utility classes.

Example:

```java
/**
 * Calculates the difference between the current encoder value
 * and the target position.
 *
 * @param targetPosition The desired encoder position.
 * @return The absolute difference between current and target.
 */
public int getPositionError(int targetPosition)
{
    return Math.abs(motor.getCurrentPosition() - targetPosition);
}
```

For internal notes or complex logic, use `//` comments sparingly.

If the comment is at least 2 lines, indent the variable or logic below it:

```java
// Placeholder value.
// Replaced once vision system initializes.
int placeholderId = -1;

// Fallback power value
double fallbackPower = 0.2;
```

Avoid obvious comments.

Bad:

```java
// Set motor power
motor.setPower(1.0);
```

---

# Documentation (.md files)

All CORE utility classes (PurpleHardware, PurpleDrive, PurpleVision, etc.) must include a:

```
{utilityName}.md
```

This file should explain:
- What the utility does
- How to initialize it
- How it interacts with OpModes
- Example usage

These files are meant for new team members.

---

# Unused Code

Do **not** leave large commented-out blocks in files.

Old code can be retrieved from Git history.  
Removing unused chunks keeps files shorter and easier to read.

---

# Imports

Place `import` statements at the top of the file in a single group.

Order:
1. Java standard library
2. FTC SDK imports
3. Third-party libraries
4. Project imports

Alphabetize within each group.

Example:

```java
import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
```

---

# Argument Formatting

Java does not support default parameters. Prefer method overloading:

```java
public void setPower(double power)
{
    setPower(power, false);
}

public void setPower(double power, boolean useSlowMode)
{
    ...
}
```

Avoid unclear null-based logic:

```java
public void setTarget(Integer position)
{

}
```

Use `null` only when it is semantically meaningful.

---

# FTC-Specific Notes

## OpModes

- Keep OpModes lean.
- Heavy logic belongs in subsystem classes.
- OpModes should coordinate, not calculate.

## Hardware Access

- Map hardware in one central location (ex: `PurpleHardware`).
- Do NOT repeatedly call `hardwareMap.get()` throughout subsystems.
- Pass references cleanly.

## Telemetry

- Do not spam telemetry.
- Group related telemetry outputs.
- Remove debug telemetry before competitions.

## State Machines

Prefer enums for robot states instead of raw integers.

```java
public enum RobotState
{
    IDLE,
    INTAKING,
    SCORING
}
```

---

# Folder Structure

Organize clearly:

```
/Components/
/Utils/ (utils that don't have a place)
/Pathing/
/Tools/
/Auto/
```

Do not mix OpModes and subsystems in the same folder.

---

# Android Studio Settings

We recommend **Android Studio** for PurpleRain development.

Enable:
- Format on Save
- Optimize imports on Save
- Reformat code on Commit
---

# Gradle & Build Hygiene

- Do not modify SDK files.
- Do not commit build artifacts.
- Keep `TeamCode` clean and modular.
- Use meaningful commit messages.

---

# Final Philosophy

PurpleRain code should be:

- Minimal
- Clear
- Intentional
- Organized
- Easy for a new team member to understand

If a variable, method, or class does not clearly serve a purpose — remove it.

Clarity > Cleverness  
Structure > Speed  
Consistency > Preference
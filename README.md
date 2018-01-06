# FTC-11792-AUTO

Our auto period file starting part.

Made by: Sambhav Saggi for FTC Team 11792

Copyright (c) FTC team 11792, all rights reserved unless otherwise explictly stated

---

## Code Description

- Turn on the color sensor light (Lines 144-146)
- Close the glyph grabber (Line 178)
- Read the VuForia (Lines 180-217)
- Knock down the jewel
  - Move down the arm (Line 221)
  - Check the color (Lines 226-233)
  - Move accordingly (Lines 235-257)

```text
If there is more red than blue and red is more than 0.5:
    If you are starting in position 1:
        Turn Counter-Clockwise
    If you are starting in position 2:
        Turn Clockwise
    If you are starting in position 3:
        Turn Counter-Clockwise
    If you are starting in position 4:
        Turn Clockwise
```

```text
If there is more blue than red and blue is more than 0.5:
    If you are starting in position 1:
        Turn Clockwise
    If you are starting in position 2:
        Turn Counter-Clockwise
    If you are starting in position 3:
        Turn Clockwise
    If you are starting in position 4:
        Turn Counter-Clockwise
```

- Move up the arm
- Turn off the color sensor light

Check the starting position set in a variable by a human.

If 1: Move accordingly

```text
Turn around
If the VuForia is left:
    Turn 3.8 inches
    Go forward 50 inches
    Turn back 3.8 inches
If the VuForia is center:
    Turn 4.15 inches
    Go forward 52 inches
    Turn back 4.15 inches
If the VuForia is right:
    Turn 4.5 inches
    Go forward 54 inches
    Turn back 4.5 inches
Release the glyph
Move back 8 inches
Close the grabber
Go forward 10 inches
```

If 2: Move accordingly

```text
If the VuForia is left:
    Turn 4.5 inches
    Go forward 54 inches
    Turn back 4.5 inches
If the VuForia is center:
    Turn 4.15 inches
    Go forward 52 inches
    Turn back 4.15 inches
If the VuForia is right
    Turn 3.8 inches
    Go forward 50 inches
    Turn back 3.8 inches
Release the glyph
Move back 8 inches
Close the grabber
Go forward 10 inches
```

If 3: Move accordingly

```text
Turn around
If the VuForia is left:
    Go forward 30 inches
If the VuForia is center:
    Go forward 38 inches
If the VuForia is right:
    Go forward 46 inches
Turn 90 degrees counter-clockwise
Go forward 6 inches
Release the glyph
Go back 8 inches
Close the grabber
Go forward 10 inches
```

If 4: Move accordingly

```text
If the VuForia is left
    Go forward 46 inches
If the VuForia is Center
    Go forward 38 inches
If the VuForia is right:
    Go forward 30 inches
Turn 90 degrees clockwise
Go forward 6 inches
Release the glyph
Go back 8 inches
Close the grabber
Go forward 10 inches
```

Notice how 1 and 2 are similar and 3 and 4 are similar. Since these starting positions are mirroring each other, code will be very similar. For your reference,

"TURN_AROUND" = 36.72 inches

"TURN_90_DEG" = 18.36 inches

---

Encoder Drive function:
How it works:
Finds where each induvidual motor has to go.

1. Gets the curent position for each motor

1. Gets the variable for how many inches it has to go, adds it to the current position.

1. Sets the encoder to go to that position

1. Turns on "Run to position": tells the motor to use the encoder

1. Sets speed to start motion.

1. Display data, stop motors when timeout occurs.

---

The glyph grabber:
Open:
rPosition = 0.60;
lPosition = 1.00;
Closed:
rPosition = 0.50;
lPosition = 0.92;

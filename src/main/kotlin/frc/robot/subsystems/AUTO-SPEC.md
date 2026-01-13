# Autonomous Specifications

## Linked waypoints

- `Lower position`: Lower starting position
- `Intake line start`
- `Intake line end`

## Paths

- `Intaking zone to high position`
    - Drive from the intaking zone, pick up a cone, wait for a second, and drive close to the second alliance's loading
      zone.
      ![image](https://github.com/Galaxia5987/Robot-template/assets/31829093/6c310a67-afb8-44f6-a652-35aaf70c5238)
    - Special constraints (if relevant)
        - Near loading zone: decrease speed so the red dragon won't notice us.
          ![image](https://github.com/Galaxia5987/Robot-template/assets/31829093/82f75dd9-9c13-4dbc-913d-741f211f3972)
- Another one...

## Autos

- `Intaking zone to high position while shooting`
    - Uses `Intaking zone to high position` path
    - Shoots after passing the second waypoint, near the Charge Station
      ![image](https://github.com/Galaxia5987/Robot-template/assets/31829093/5e5d017e-a853-4621-8156-03d409aa8542)
- Another one...

# Choosers

> Could be as simple as "Using PathPlanner's built-in chooser" (BTW
> use [this](https://www.chiefdelphi.com/t/pathplanner-2024-beta/442364/149?u=dan)) or more complex based on the game,
> like what's been
>
done [here](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2023-build-thread/420691/179#autos-the-questionnaire-2):

## Side chooser

Choose which side the robot's on:

- Left
- Center
- Right

## Game piece chooser

Choose which game piece type the robot is preloaded with:

- Cube
- Cone

## State machine

> Mermaid state graph which shows which auto to run based on the chooser goes here.

# Dashboard

> Include here fields that will be sent over NetworkTables and shown to the drivers during the autonomous period of the
> match.

- Robot location: `Robot/Odometry`
- Time left until teleop starts: `Robot/TimeLeftUntilTeleop`

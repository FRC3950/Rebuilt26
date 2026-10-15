# Chuck Norris: a rookie’s shot-tuning guide

**Your job: try a setting, shoot a few balls, and write down how many go in.**

You do not need to measure how fast the ball flies or how fast it spins. You only need the robot, its dashboard, a few balls, and the shot sheet.

## 1. Learn the words on the dashboard

| Word | What it means |
|---|---|
| Hub | The goal you are shooting into. |
| Turret | The part of the shooter that turns to aim. Chuck has a left and a right turret. |
| Trim | A small change added to an automatic setting. |
| Hood | The part that changes the ball’s launch angle. You enter its setting in degrees. |
| RPS | Wheel turns per second. A higher wheel-RPS setting asks the shooter wheels to turn faster. This is not a measurement of ball speed. |
| Manual Tune | You choose the hood and wheel settings on the dashboard. |
| Automatic mode | The robot chooses the shot settings based on its position and motion. |
| Speed multiplier | A number that adjusts the wheel setting in automatic mode. A larger number asks for faster wheels. |

The **dashboard** is the screen with the robot’s settings. The **model** is the saved set of numbers the robot uses to choose a shot.

## 2. Prepare the robot once

Use your team’s normal startup and shooting-area rules.

Have a teammate check that the robot knows where it is and points toward the hub. Choose a flat place around the middle of your shooting range. Mark the parking spot and the direction the robot faces.

Measure or estimate the launch angle at these two hood settings:

- Hood set to **13°**.
- Hood set to **29.85°**.

The **hood setting** and the **ball’s launch angle** are different numbers. Do not copy one into the other by mistake. Write down whether your launch angles were measured or estimated.

Have the programming teammate enter those angles on the map page and in the matching dashboard fields. They must build and load the matching model once. You do not need a new model after every test shot.

For a fresh test, use these starting settings:

| Dashboard setting | Start with |
|---|---:|
| `ShotModel/Left/Speed Multiplier` | 1 |
| `ShotModel/Right/Speed Multiplier` | 1 |
| Left and right `Hood Trim Deg` | 0 |
| Left and right `Aim Trim Deg` | 0 |
| `ShotModel/Advanced/Release Delay Sec` | 0 |

Leave drag, spin, lift, and spin decay at **0**. Keep the current ball-speed-transfer and map variation settings for now. You will tune by watching shots, not by trying to measure each of these effects.

If you are continuing an earlier test, write down the saved settings before changing them.

## 3. Write down the robot’s starting settings

Park on your marked spot and stay still.

Set `Tune/Use Tune Values` to **OFF**. This is automatic mode.

Choose which turret’s shots you will watch. For the left turret, write down:

| Copy this dashboard value | Call it this on your sheet |
|---|---|
| `ShotModel/Left/Calculated Hood Deg` | Starting hood |
| `ShotModel/Left/Calculated Wheel RPS` | Starting wheel setting |
| `ShotModel/Left/Speed Multiplier` | Current speed multiplier |

For the right turret, use the same names with **Right** instead of **Left**.

Check `ShotModel/Left/Model Valid`, or the Right version. It must be **true** before you use its numbers. The starting wheel setting must be greater than zero.

If the model is invalid, tell the programming teammate what the status message says. Do not copy zero or old settings and treat them as a good starting shot. Some places have gaps in the current model. A gap does not prove the robot cannot make that shot.

**Write these numbers down before turning Manual Tune on.** The automatic numbers may stop updating while you use Manual Tune.

## 4. Try nearby wheel settings

Set `Tune/Use Tune Values` to **ON**. This is Manual Tune.

1. Put your **starting hood** into `Tune/Hood Deg`.
2. Put your **starting wheel setting** into `Tune/Flywheel RPS`.
3. Keep the robot parked in the same place, facing the same direction.
4. Shoot **three balls**. Write down how many score.

For this test, a **clean make** goes in without touching the rim. Write rim touches in a separate column if you can see them.

If the setting does not work well, try a little higher and a little lower. Keep the hood the same.

**Example search only:** if the starting wheel setting is 40 RPS, test these settings in order:

| Test | Wheel setting |
|---|---:|
| First | 40 RPS |
| Next | 42 RPS |
| Next | 38 RPS |
| Next | 44 RPS |
| Next | 36 RPS |

Use your own starting value. You do not need to tell whether a miss was short or long. Just count makes. If you can tell the miss direction, that may help you choose the next test faster.

Keep wheel settings within **0–100 RPS** and the team’s normal test range. Keep hood settings within **13–29.85°**.

The two manual settings control **both turrets**. Record the left and right results separately. Do not add their scores together. Use the team’s normal controls to run the test as needed; these are not separate left/right manual widgets.

**Stay still in Manual Tune.** This mode does not correct the shot for robot movement. It also does not add the automatic mode’s side trims.

## 5. Find a setting with room for error

When a wheel setting starts working:

1. Try nearby settings in **0.5–1 RPS** steps.
2. Shoot **five balls** at each setting.
3. Write each result as makes out of attempts, such as **4/5**.

Prefer settings that make 5/5. A 4/5 result is worth testing again. One good shot is not enough to choose a setting.

Look for several wheel settings **next to each other** that keep working. Pick the middle of that group and test it again.

For example, if 52, 54, and 56 RPS all work after checking the settings between them, try 54 RPS. These are example numbers, not settings already tested on Chuck.

Do not assume every number between two successful tests works. Check the numbers in between. If two groups work with a group of misses between them, keep those groups separate.

### What if none of the wheel settings work?

Keep one wheel setting fixed. Try the starting hood **plus 1°**, then **minus 1°**, within the hood limits. Try nearby wheel settings again if the hood change helps.

Change **one setting at a time** so you know which change helped. Write down both the hood and wheel setting that worked.

If every ball goes to the same side, have a teammate check the aim and robot position. Faster wheels will not fix a shot that points the wrong way.

Also check the dashboard’s measured wheel RPS. If the wheels cannot reach or hold the requested speed, have the team fix that first. You do not need a separate speed-measuring tool.

## 6. Use your good setting in automatic mode

The calculator near the top of this page does the math for you.

Enter:

1. **Starting wheel setting:** the automatic value you wrote down in Step 3.
2. **Best wheel setting:** the manual value you chose in Step 5.
3. **Current speed multiplier:** the value you wrote down in Step 3.

Copy the answer into the turret you tested:

- `ShotModel/Left/Speed Multiplier`, or
- `ShotModel/Right/Speed Multiplier`.

The calculator does **not** send anything to the robot. You must copy the answer to the dashboard yourself.

On a printed copy, you can do the same math: multiply the current multiplier by the best wheel setting, then divide by the starting wheel setting.

**Example only:** starting wheels 40 RPS, best wheels 55 RPS, and current multiplier 1 gives a new multiplier of **1.375**.

### If you also changed the hood

If the old hood trim was zero, subtract the starting hood from the hood that worked. Enter that difference in the side’s `Hood Trim Deg` field.

Example only: starting hood 18°, good hood 19° means a hood trim of **+1°**.

If the old hood trim was not zero, add this difference to the old trim. Have a teammate check the numbers if you are unsure.

Now set `Tune/Use Tune Values` to **OFF**. Stay on the same parking mark. Shoot five more balls in automatic mode.

Check that the dashboard commands match the shot you meant to test. If they do not, check your parking spot, copied numbers, and mode switch.

Repeat the process for the other turret. It may need a different adjustment.

## 7. Check near, middle, and far

Keep the same automatic adjustments while you test all three places.

1. Shoot five balls from your middle spot.
2. Shoot five from a marked near spot.
3. Shoot five from a marked far spot.
4. Return to the middle spot and check it again.

Use places where the model is valid and the robot has a clear shot. Record each turret’s results on its own sheet.

| What happens? | What to do next |
|---|---|
| All three places work | Keep the settings. Test more balls and more places. |
| One place fails | Use Manual Tune there to find a setting that works. Record it, then turn Manual Tune off again. |
| Each place needs a different multiplier | Keep the results. Ask the programming teammate to use them to improve the model. |
| The left and right turrets need different settings | Keep separate side adjustments. Test both at all three places. |
| A manual shot works where the model is invalid | Record the shot and the error message. The model needs work at that place. |

Do not keep changing the same multiplier for each parking spot and call the last value “finished.” You want one saved set of adjustments that works at several distances.

Five shots are a quick check. Before calling the settings ready, repeat with more shots, several balls, and more field positions. Even five makes in a row do not prove the shot will always work.

## 8. Save the settings

After an improvement passes your checks, click `ShotModel/Save Settings`.

This keeps the dashboard settings after a robot restart. It does **not** build or load a new model file.

Give the programming teammate your shot sheet when the settings work at several distances. They can update the model’s overall speed correction and build a new map if needed. After they do, retest the shots. Do not apply the same correction twice in the model and the side multiplier.

You do not need to measure ball speed for this step. The correction comes from the settings that made shots.

## 9. Test moving shots last

Only start this step after parked shots work.

Set `Tune/Use Tune Values` to **OFF**. Use automatic mode for all moving tests.

Test:

- Slowly toward the hub, then away from it.
- Slowly across the hub in both directions.
- Faster motion, then up to the **2.5 m/s** shooting limit where the shot is valid.
- Chassis turns in both directions, watching each turret separately.

Keep the parked-shot settings unchanged at first.

If moving shots need work, try a small change to `ShotModel/Advanced/Release Delay Sec` with the team. This setting tells the robot how far ahead to predict the release point. It is **not** how long the ball flies.

For example, compare the current delay with a value **0.01 second higher**. Test both travel directions at each value. Keep the change only if the overall results improve. If it helps one direction but hurts the other, go back and record what happened.

Delay starts at zero and cannot be negative in this version. If delay changes do not help, ask the team to check the model and motion readings. Do not keep increasing it without better results.

After level moving shots work, test the pitch and roll conditions your team expects. Pitch means the robot leans forward or back. Roll means it leans side to side.

## 10. Know what this procedure can do

You are finding settings that work by testing shots. You are **not** measuring each cause of a miss.

Keep drag, lift, spin, and spin decay off for the first tests. You do not need to work out each one separately.

If different distances need different corrections, keep the hood, wheel, and make-count results. Those results can guide the next model update.

The app does **not yet build a new model just from make/miss rows**. Do not fill in guessed ball speeds in the older measurement page. Give the team your real shot results instead.

### What goes on the sheet?

- Where you parked and which turret you watched.
- Starting hood, starting wheels, and current speed multiplier.
- The hood and wheel settings you tried.
- Clean makes and total attempts.
- The adjustment you kept and whether it also worked at other places.

There is no ball-speed or spin-measurement column. Download the blank shot sheet near the top of this page. Open it in a spreadsheet app to fill it in or print it.

**The goal:** one saved set of adjustments that works when parked at several distances, then works during moving tests.

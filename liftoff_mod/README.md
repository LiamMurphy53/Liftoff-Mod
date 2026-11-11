# GoPro Geometry Lab for Liftoff

Experimental Mac mod for comparing the GoPro Drone's deadcat motor layout inside Liftoff. It changes the **actual propeller force application points**, rigid-body mass, center of gravity, and full inertia tensor. Apply also installs a geometry-aware motor allocator and adjusts Liftoff's native controller gains to the new body-axis inertia. The selected propulsion and stick rates remain Liftoff's.

This is a working development prototype, not a calibrated simulation of the finished aircraft. Read the limitations below before using its handling to make design decisions.

## Install on another computer

**The installer currently supports macOS only.** It has been tested on an Apple Silicon Mac. Setup also includes Intel Mac dependencies, but an Intel Mac installation has not been tested. Windows needs its own loader/launcher and a compatibility check against the Windows game files; these Mac scripts will not install it on Windows.

### 1. Prepare the Mac

- Install your own copy of **Liftoff through Steam**. Launch it normally once, then quit the game. Keep Steam open and signed in.
- The supported game is **Liftoff 1.7.6 Mac, Steam build 25599583**. The mod checks the exact game assembly, not just the displayed version. A different build disables the mod until its compatibility is checked; rebuilding alone does not make a newer game compatible.
- Install **Python 3** if needed, using the [official macOS installer](https://www.python.org/downloads/macos/). In Terminal, `python3 --version` should report Python 3. Setup does not install Python for you.
- On **Apple Silicon**, the launcher needs Rosetta to run the game in Intel mode. Follow [Apple's Rosetta instructions](https://support.apple.com/en-ca/102527) if it is not available. An Intel Mac does not need Rosetta.
- Keep an internet connection available for the first setup. The scripts download their own .NET SDK and BepInEx loader; you do not need to install those separately.

### 2. Download and build the mod

1. Open [the Drone-sim repository](https://github.com/LiamMurphy53/Drone-sim), choose **Code → Download ZIP**, and unzip it. You can also clone the repository with Git.
2. Keep the extracted project in a folder you can write to, such as Documents. Open its **liftoff_mod** folder. You do not need to set up the separate `flight_sim` or `matlab_sim` folders to use this mod.
3. Double-click **Setup Geometry Lab.command**. A Terminal window opens while setup downloads the dependencies, checks their hashes, builds the plugin against your installed Liftoff, and runs the math tests.
4. Wait for **“Geometry Lab built and staged.”** If setup reports an error, use the troubleshooting section below before launching.

If double-clicking the setup file does not run it, open Terminal, type `cd ` (including the space), drag the **liftoff_mod** folder into the window, and press Return. Then run:

```sh
python3 tools/setup_mac.py
```

Run all commands in this guide from inside **liftoff_mod**. The repository contains source code, not the downloaded tools or a ready-built plugin. First setup creates these locally in `.tools/`; you do not need to copy them from another computer. Setup does not modify `Liftoff.app`, Steam launch settings, or global .NET installations.

### 3. Connect the radio and launch

1. Connect the radio before launching. For the **RadioMaster Pocket**, use the **top USB-C data port**, choose **USB Joystick** on the radio, and configure/calibrate it in Liftoff's controller settings.
2. Keep Steam open and signed in, and quit any already-running Liftoff instance.
3. Double-click **Launch Liftoff Geometry Lab.command**. From Terminal, the equivalent is:

   ```sh
   python3 tools/launch_mac.py
   ```

4. Acknowledge Liftoff's external modifications notice. The game disables certain competitive features while mods are loaded; this mod does not alter that protection.
5. Open **Single Player → Free Flight**, choose a four-motor drone using **ZetaFlight**, and leave it stopped on the ground. Acro is the intended test mode. A DJI FPV configuration has also passed the integration check, but the selected drone's propulsion and drag are still inherited.
6. The **Geometry Lab** panel sits top left (**F8** hides it). Click **Apply** to fly the active profile, or **Open drone editor** to set up your own drone in the browser (see [The drone editor](#the-drone-editor)). Apply installs the geometry, mass properties, motor balancing, thrust curve, camera and CAD body together.
7. Fly. The panel's status turns green (**LIVE**) when the game applies forces through the modified motors; it confirms the mod is active, not real-world calibration.

### Subsequent launches and updates

Use **Launch Liftoff Geometry Lab.command** each time you want the mod. Setup is only needed for the initial installation or after updating the mod source. To rebuild an updated checkout using already-downloaded tools:

```sh
python3 tools/setup_mac.py --build-only
```

Quit and relaunch Liftoff to load the rebuilt plugin. If `.tools/` is missing (for example, you downloaded a fresh ZIP into a new folder), run setup without `--build-only`.

**A drone reset returns to stock, then the mod applies your profile again once the drone is still** (turn off **Re-apply after resets** to stop that). **Stock** returns the original geometry, mass properties, controller, camera and frame. Press **F8** to hide the panel. With the panel open, **F9** applies and **F10** restores while stopped. Liftoff also uses F9 for its PID graph, so the on-screen Apply button avoids toggling that graph. On keyboards that use the top row for media controls, use **Fn/Globe + F8**, etc., as needed.

To run without the mod, quit the game and launch Liftoff normally from Steam. No installed game files need to be removed.

### Liftoff installed in another Steam library

The scripts normally look for:

```text
~/Library/Application Support/Steam/steamapps/common/Liftoff/Liftoff.app
```

If Liftoff is elsewhere, replace `/path/to/Liftoff.app` below with its actual path. Run **both** setup and launch with that path:

```sh
python3 tools/setup_mac.py --game "/path/to/Liftoff.app"
python3 tools/launch_mac.py --game "/path/to/Liftoff.app"
```

The custom path is not saved by the double-click launchers. Keep using `--game` for later launches and rebuilds, for example:

```sh
python3 tools/setup_mac.py --build-only --game "/path/to/Liftoff.app"
```

### Installation troubleshooting

| Message or symptom | What to do |
| --- | --- |
| `python3` missing or setup cannot start Python | Install Python 3 using the link above, reopen Terminal, and check `python3 --version`. |
| Liftoff installation not found | Install the game through Steam, or supply its actual location with `--game` for both setup and launch. |
| Local tools or plugin missing | Run setup without `--build-only` and wait for the success message. |
| Rosetta required | Follow Apple's Rosetta instructions above, then retry the launcher. |
| Liftoff already running | Quit the current game, then use the mod launcher. It cannot attach to an existing session. |
| Steam is starting | Sign in, leave Steam open, and run the mod launcher again. |
| Unverified/unsupported Liftoff build | This game assembly needs a new compatibility check. Keep the version guard enabled; reinstalling dependencies does not resolve this. |
| Apply unavailable or asks you to stop | Enter single-player Free Flight with a supported ZetaFlight quad, reset it, and let it settle before applying. |
| No panel in Free Flight | Press F8; if it is still absent, check the plugin log below and confirm you used the mod launcher. |
| Drone editor says "Liftoff not running" | The editor talks to the running game. Launch with the mod launcher, then reload the page. If port 8737 was taken, the plugin log names the port it used. |
| Pocket sticks do not respond | Confirm the top data port and USB Joystick mode, check Liftoff's calibration, and reload Free Flight after reconnecting. |

For a setup failure, keep the error text in the setup Terminal window. For a launch failure, check `runtime/launcher.log` and `runtime/player.log`. Plugin status and physics checks are in `.tools/bepinex/BepInEx/LogOutput.log`. These paths are all inside `liftoff_mod`.

## The drone editor

**Open drone editor** in the panel opens `http://127.0.0.1:8737/` in your browser. The page is served by the plugin, only to this computer, and works while Liftoff is running. **Apply to drone** saves and flies the profile; **Save** only saves it; **Stock** puts the Liftoff drone back. The pill at the top shows whether the drone is bound, applied and live.

1. **Model.** Drop in an **STL, OBJ, GLB or glTF** (from Onshape: right-click the part studio or assembly → *Export*). Set the file's **units**, **up axis** and **forward axis** until the green arrow on the ground points out the front. The editor guesses millimetres for STL/OBJ and meters, +Y up for glTF. The model replaces Liftoff's stock frame in the game, unless you untick **Hide Liftoff's stock frame**; Liftoff's motors and props stay and move to your motor points. Use **One colour** to override the CAD colours.
2. **Motors.** Click **Pick** next to a motor, then click the top of that motor on the model. With **Rim** ticked, click three points around the bell's edge and the centre is used. You can also type coordinates. **Auto-label** sorts the four points into front-left, front-right, rear-left and rear-right from the forward axis. The prop size only draws the prop discs.
3. **Mass & CG.** Type the mass and centre of mass from your CAD mass properties, in the model's own coordinates and units (no conversion needed), or pick the CG on the model. Enter the inertia tensor about the CG, in the CAD axes, in any of the listed units.
4. **Thrust.** Tick **Use my measured thrust curve** to replace Liftoff's propeller thrust with one motor's static thrust against throttle. **Import thrust stand CSV** reads the [thrust stand](https://github.com/LiamMurphy53/thrust-stand) sweep export (`throttle_pct`, `thrust_g`) or a raw log (binned per 1% throttle, median). You can also edit the table. The curve must reach 100% throttle; **Extend to 100%** extrapolates from the last two points if you didn't measure it. The chart shows the curve, the hover line (weight ÷ 4), hover throttle and thrust-to-weight.
5. **Drag.** Pick where the body drag comes from:
   - **Liftoff** keeps the game's own drag.
   - **CAD shape × Cd** measures your model's silhouette area from 162 directions and multiplies it by one drag coefficient. Quad bodies are about 1.0–1.3.
   - **CFD runs** uses your OpenFOAM (or other CFD) results. Add one run per flow direction:
     - **U** is the freestream velocity you set in `0/U`, in your CAD axes.
     - **F** is the total force (pressure + viscous) from the `forces` function object.
     - **Import force.dat or CSV** reads `postProcessing/<forces>/<time>/force.dat` or `forces.dat`. Both the OpenCFD column layout and the bracketed Foundation layout work. The force is averaged over the last fifth of the run, and a run that's still moving is flagged. You then type that run's U.
     - A summary CSV with `name, ux, uy, uz, fx, fy, fz` columns imports many runs at once.

   **Mirror left ↔ right** reuses each run for its mirror image. Between runs, the drag follows the model's silhouette, scaled to the drag coefficient your runs imply nearby. With no model loaded, it blends between the runs. The plot shows drag area against direction of travel from the side and from above, with Liftoff's current drag dashed for comparison. Pink arrows in the 3D view show each run's airflow, so a wrong axis is easy to spot.
6. **FPV camera.** Tick **Place the camera from this model**, pick the front of the lens, and set the uptilt. Uptilt is measured from the motor plane, which is flat when the drone hovers level.

Every number in the editor is in the CAD file's coordinates. The profile itself (`active.json`) stores meters, kilograms and kg·m² in **X left, Y backward, Z up**, with motors in **FL, FR, RL, RR** order. The editor re-centres it on the motor centre; only positions relative to each other matter. The body mesh is saved beside it as `body.glmb`. Both live in `.tools/bepinex/BepInEx/config/GoProGeometry/`. Building does not overwrite an existing active profile. Copy a tuned profile into `profiles/` if you want to commit it.

The supplied `profiles/gopro-drone.json` imports the original model's 808.819 g mass, CAD motor coordinates, CG, and inertia from `flight_sim/config/aircraft.json`. It includes the off-diagonal inertia terms. It has no thrust curve, camera or body model, so Liftoff's propulsion and camera are kept until you add them.

The adapter converts to Unity right/up/forward, aligns the average motor center to the stock drone's motor center, and shifts CG, camera and body by the same offset. This preserves the physically relevant motor-to-CG vectors. Principal-axis decomposition converts the full inertia tensor into Unity's tensor plus rotation.

**Moving motors does not automatically recompute mass, CG, or inertia.** Update those from CAD when evaluating a changed frame. For a controlled comparison you can hold them fixed, but that isolates lever-arm effects rather than predicting the entire redesigned aircraft.

### How drag is applied

Liftoff's body drag is ½ · 1.184 · Cd · A · v², applied against the drone's velocity through the air (wind is subtracted first). Cd is the frame's `DragCoefficient`. A is the drone's silhouette area, which Liftoff measures from 42 directions round the drone when it builds it. In flight, Liftoff blends the three nearest directions. Liftoff adds two more terms of its own:

- drag on the spinning props from sideways airflow;
- a small downward push.

There's no lift, side force or moment from the body, and rotation is damped by Unity's angular drag rather than by aerodynamics.

The mod keeps all of that and swaps only the area table. For each of Liftoff's 42 directions, it writes your Cd·A for that direction divided by the frame's Cd. Liftoff's force then equals your drag area exactly, with its wind and blending unchanged. Prop drag is still Liftoff's, so leave the props out of CFD models.

When setting up CFD:
- Rotate U rather than the model, and use freestream boundary conditions on the outer faces, so one mesh serves every direction.
- One speed (15–25 m/s) is enough; a bluff body's Cd barely changes with speed.
- A forward-flying quad is pitched nose-down, so in normal flight the air hits it **from the front and above**. Prioritise that sweep (0°, 15°, 30°, 45°, 60° above the motor plane), then straight above and below, the side, and a couple of diagonals.

### How the thrust curve is applied

Liftoff sets each motor's RPM from the throttle stick (throttle × max RPM), then gets thrust from its propeller model at that RPM and the airspeed through the prop. With a thrust curve, the mod reads the throttle back out of Liftoff's collective. It then sets collective thrust to four times your measured thrust at that throttle, scaled by Liftoff's own ratio of in-flight to static thrust, so airspeed losses still apply. Each motor is limited between your curve's thrust at idle and at 100%. The allocator then balances pitch and roll around the real CG as before. The throttle percentages are treated as the same as the stick's throttle, which is how Betaflight's motor output relates to throttle in Acro before mixing.

## What it models—and what remains approximate

| Item | Current behavior |
| --- | --- |
| Motor lever arms | Actual Liftoff propeller transforms and force locations change |
| Mass, CG, inertia | Original model values applied to the flight Rigidbody; full tensor retained |
| Radio input and rates | Liftoff's selected ZetaFlight configuration; not real Betaflight firmware |
| Pitch/roll mixer | Allocates bounded motor forces from actual motor-to-CG arms, with automatic hover balance |
| Yaw | Liftoff's simplified direct yaw torque remains; allocator balances opposite spin pairs |
| Thrust | Your measured static thrust curve when set (with Liftoff's airspeed losses); otherwise the stock propulsion |
| Motor response, prop model | Inherited from the stock Liftoff build you selected |
| FPV camera | Lens position and uptilt from the profile when set; otherwise Liftoff's |
| Body mesh | Your CAD model when set, replacing the stock frame's look |
| Body drag | Your Cd·A against direction (CAD silhouette × Cd, or CFD runs) when set, through Liftoff's own drag; otherwise Liftoff's. Still drag-only: no body lift, side force or moment |
| Collision geometry | Still the stock frame's |
| Controller gains | Native PID objects rebuilt with body-axis inertia scaling; auto mode adds bounded pitch/roll integral correction |
| Windows | Source is portable C#, but loader setup and assembly compatibility remain untested |

The selected stock propulsion and the simplified controller can dominate handling. This version lets us establish whether custom geometry reaches the game's physics, and explore pitch/roll lever arms. It does **not** yet establish that real Betaflight on the GoPro Drone will behave the same way, especially near saturation or in yaw. Your motor/prop thrust data and real flight/blackbox comparisons are still needed for calibration. Real reaction-torque behavior, body lift and moments from CFD, and custom collision geometry remain follow-up work.

## Apply now includes controller compensation

The imported CG is forward of the average motor center. Equal motor thrust creates a nose-down moment; this is not proof that a controlled deadcat must continually pitch forward. Apply solves for the thrust needed at each real force location. The supplied profile needs approximately **63.77% front / 36.23% rear** in level hover. The physical CG and lever arms are not moved to hide this imbalance.

Pitch and roll commands use the same force allocation. Each motor remains between the selected idle force and the game's available maximum. Collective is adjusted to preserve attitude control where possible; impossible pitch/roll requests are scaled together. A forward CG therefore still uses more front-motor headroom and can reduce maximum balanced collective thrust. Profiles that cannot balance positive motor thrust are rejected.

Native ZetaFlight PID gains are rescaled using the full tensor's **body-axis** diagonal, rather than mistaking principal moments for body-axis moments. Existing manual gain proportions are retained. In the game's P-only auto mode, pitch/roll gain I is set to twice P (0.5 s integral time) to reject remaining steady disturbances. Stored integral correction is limited to a moment equivalent to a 20 mm load offset and stops accumulating further into mixer saturation. Input rates/expo and flight-mode selection are unchanged. This is an experimental controller adapter, not a Betaflight firmware implementation or a measured real-aircraft tune.

The adapter replaces the final motor force/RPM outputs before Liftoff applies its own propeller forces and airflow effects. Liftoff's earlier throttle/headroom policy remains in place, so it can be more conservative than the allocator alone. Normal Acro flight is the intended test mode; special launch-assist branches bypass ordinary motor mixing and are not modeled by this correction. Liftoff still supplies the simplified yaw torque independently of motor reaction torque.

**Restore** puts back the original native PID dictionary along with the physical geometry. Changing controller settings while the profile is active disables the override; stop and Apply again. Reset still returns to stock. Neither Apply nor Restore writes over the saved Liftoff drone configuration.

## Compatibility and validation

Target inspected: **Liftoff 1.7.6 Mac**, Steam build **25599583**, Unity **2022.3.62f3**, game assembly MVID `b268710e-07ea-4416-8783-dcbcddcadb82`. (Earlier versions of the mod targeted 1.7.5, build 25118475.) A version guard disables changes on other game assemblies instead of guessing at obfuscated internals. Do not remove that guard without inspecting and testing the new build.

### Porting to a new Liftoff update

Liftoff's code is obfuscated, and every update renumbers the internal members the mod hooks. The inspector finds them again by structure and regenerates `src/GameTokens.cs`, which also holds the version guard. Each match must be unique, or it stops with an error. Run this from `liftoff_mod` after the update:

```sh
export DOTNET_CLI_HOME="$PWD/.tools/cli-home"
.tools/dotnet/dotnet build tools/Inspector/Inspector.csproj -c Release
.tools/dotnet/dotnet tools/Inspector/bin/Release/net8.0/Inspector.dll \
  ~/Library/Application\ Support/Steam/steamapps/common/Liftoff/Liftoff.app/Contents/Resources/Data/Managed/Assembly-CSharp.dll \
  locate src/GameTokens.cs "Liftoff X.Y.Z Mac, Steam build NNNNNNNN"
python3 tools/setup_mac.py --build-only
```

Then launch, confirm the plugin log says `Geometry Lab ready` with no errors, and apply the profile in Free Flight before trusting the port. If the locator fails, the game changed shape, not just numbering: inspect it by hand.

Validated during development:

- Release compilation with zero warnings/errors.
- 22 math checks: coordinate orientation, full tensor, invalid geometry, CG-balanced hover, moment signs, full-throttle headroom, saturation, reverse thrust, symmetric geometry, and a sweep of 1,000 bounded throttle/attitude requests.
- New isolated controller check using the game's native PID and actual propeller force method with the GoPro full inertia tensor: neutral throttle steps produced only 0.000045 rad/s peak numerical rotation; an initial 2 rad/s pitch plus 1 rad/s roll disturbance settled below 0.01 rad/s; a persistent 0.02 Nm pitch disturbance was rejected. These isolate controller/force correctness and do not include the selected drone's drag, wind, or RC input.
- Loaded in the Mac game and displayed the editor in Free Flight.
- Applied the GoPro geometry to a loaded drone: mass changed from 0.795000 to 0.808819 kg; the CG, full inertia, and four motor positions changed. A subsequent reset returned stock values. World/local motor readback was within 0.1 mm in this test; the live guard allows 1 mm for large-map floating-point roundoff.
- Called **Liftoff's actual `Propeller.ApplyControllerForceAtPropeller`** in an isolated Unity physics scene. At 1 N for 2 ms, a 100 mm lateral arm produced 0.050000 rad/s; a 200 mm arm produced 0.100000 rad/s. A forward arm gave the expected pitch sign and magnitude. No game drone or global simulation settings were changed by this test.
- **Controller adapter integration, September 18, 2026:** Apply installed the new auto PI controller and geometry allocator on the loaded 0.795 kg DJI FPV configuration. The log confirmed native powered motor calls after 55 geometry-mixer steps, with FL/FR/RL/RR hover shares 31.89/31.88/18.12/18.11%. Restore returned the stock controller, 0.795 kg mass and original mass properties; applying again restored the corrected GoPro configuration. The initial Apply attempt was correctly refused while the body had not settled.
- **Powered Free Flight smoke check, September 18, 2026 (before controller adapter):** launched the GitHub repository build with the RadioMaster Pocket connected. Liftoff detected `Radiomaster Pocket Joystick`; the user confirmed arming and flight, and the view showed the drone airborne. After the GoPro profile was applied, the plugin logged `Verified live Liftoff propeller-force calls using the custom motor positions.` A subsequent reset rebound the stock 0.795000 kg drone and its original mass properties. No geometry-guard errors were logged during this check.

This confirms integration during powered flight; **it does not validate real-world handling accuracy**. A powered user comparison of the new controller compensation and calibration against the real aircraft remain outstanding. Motor-allocation saturation is tested; real yaw authority is not. Resets clear the override: reapply the profile before each GoPro test flight. Automated mouse input was unreliable on this Mac; keyboard shortcuts are provided as a fallback.

Logs:

- `runtime/player.log`: Unity/game log.
- `.tools/bepinex/BepInEx/LogOutput.log`: loader/plugin log and force check.
- `.tools/bepinex/BepInEx/config/GoProGeometry/last-applied.txt`: actual applied mass properties and motor positions, when applied.

If startup stalls, check Steam is signed in and focus the Liftoff window. If the game is already open, quit before relaunching. The isolated loader uses `HideManagerGameObject=true` to retain the plugin across scene cleanup; Liftoff's modification detection remains active.

## Dependencies and source inspection

- [BepInEx 5.4.23.5](https://github.com/BepInEx/BepInEx/releases/tag/v5.4.23.5), universal Mac loader.
- [Microsoft .NET 8 SDK](https://dotnet.microsoft.com/download/dotnet/8.0), pinned to 8.0.425.
- Unity/Liftoff assemblies referenced from the owner's installed game; **not redistributed**.

`tools/Inspector` is an optional read-only Mono.Cecil developer utility for finding force-related call sites and, with `locate`, regenerating the game hooks after an update. Proprietary inspection output, logs, downloads, compiled binaries, and active profiles are ignored by Git. Only our adapter, math, tests, launcher, profile, and documentation belong in the repository.

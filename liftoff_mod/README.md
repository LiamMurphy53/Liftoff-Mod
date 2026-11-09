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
6. Open the editor with **F8** or its top-left button. Click **Apply GoPro Drone / edited geometry**. Apply installs the geometry, mass properties, motor balancing, and controller compensation together.
7. Fly. The editor's **LIVE** message appears when the game applies forces through the modified motors; it confirms the mod is active, not real-world calibration.

### Subsequent launches and updates

Use **Launch Liftoff Geometry Lab.command** each time you want the mod. Setup is only needed for the initial installation or after updating the mod source. To rebuild an updated checkout using already-downloaded tools:

```sh
python3 tools/setup_mac.py --build-only
```

Quit and relaunch Liftoff to load the rebuilt plugin. If `.tools/` is missing (for example, you downloaded a fresh ZIP into a new folder), run setup without `--build-only`.

**A drone reset returns to stock; click Apply again after each reset.** **Restore stock drone** returns the original geometry, mass properties, and controller. Press **F8** to hide the editor. With the editor open, **F9** applies and **F10** restores while stopped. Liftoff also uses F9 for its PID graph, so the on-screen Apply button avoids toggling that graph. On keyboards that use the top row for media controls, use **Fn/Globe + F8**, etc., as needed.

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
| No editor in Free Flight | Press F8; if it is still absent, check the plugin log below and confirm you used the mod launcher. |
| Pocket sticks do not respond | Confirm the top data port and USB Joystick mode, check Liftoff's calibration, and reload Free Flight after reconnecting. |

For a setup failure, keep the error text in the setup Terminal window. For a launch failure, check `runtime/launcher.log` and `runtime/player.log`. Plugin status and physics checks are in `.tools/bepinex/BepInEx/LogOutput.log`. These paths are all inside `liftoff_mod`.

## Editing the drone

The supplied `profiles/gopro-drone.json` imports the original model's 808.819 g mass, CAD motor coordinates, CG, and inertia from `flight_sim/config/aircraft.json`. It includes the off-diagonal inertia terms.

The live editable copy is `.tools/bepinex/BepInEx/config/GoProGeometry/active.json`. The overlay opens its containing folder. **Save edits** saves there; **Reload profile JSON** reads it; **Apply** changes the current drone. Building does not overwrite an existing active profile. Copy a tuned profile into `profiles/` if you want to commit it.

The editor shows millimeters and grams. JSON uses meters, kilograms, and kg·m². Its motor array is flattened in **FL, FR, RL, RR** order, each with **X left, Y backward, Z up**. The inertia array is row-major, about the CG, in those same CAD axes.

The adapter converts to Unity right/up/forward, aligns the average motor center to the stock drone's motor center, and shifts CG by the same offset. This preserves the physically relevant motor-to-CG vectors. Principal-axis decomposition converts the full inertia tensor into Unity's tensor plus rotation.

**Moving motors does not automatically recompute mass, CG, or inertia.** Update those from CAD when evaluating a changed frame. For a controlled comparison you can hold them fixed, but that isolates lever-arm effects rather than predicting the entire redesigned aircraft.

## What it models—and what remains approximate

| Item | Current behavior |
| --- | --- |
| Motor lever arms | Actual Liftoff propeller transforms and force locations change |
| Mass, CG, inertia | Original model values applied to the flight Rigidbody; full tensor retained |
| Radio input and rates | Liftoff's selected ZetaFlight configuration; not real Betaflight firmware |
| Pitch/roll mixer | Allocates bounded motor forces from actual motor-to-CG arms, with automatic hover balance |
| Yaw | Liftoff's simplified direct yaw torque remains; allocator balances opposite spin pairs |
| Thrust, motor response, prop model | Inherited from the stock Liftoff build you selected |
| Body mesh, drag, collision geometry | Stock frame remains; motor/prop assemblies move, arms are not remodeled |
| Controller gains | Native PID objects rebuilt with body-axis inertia scaling; auto mode adds bounded pitch/roll integral correction |
| Windows | Source is portable C#, but loader setup and assembly compatibility remain untested |

The selected stock propulsion and the simplified controller can dominate handling. This version lets us establish whether custom geometry reaches the game's physics, and explore pitch/roll lever arms. It does **not** yet establish that real Betaflight on the GoPro Drone will behave the same way, especially near saturation or in yaw. Your motor/prop thrust data and real flight/blackbox comparisons are still needed for calibration. Real reaction-torque behavior and custom drag/collision geometry remain follow-up work.

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

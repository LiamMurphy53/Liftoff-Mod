# Drone Sim

## Install the Liftoff GoPro Drone mod

**macOS only for now. Windows installation is not supported yet.** Tested with Liftoff **1.7.6 Mac, Steam build 25599583**; the mod refuses other game assemblies until their compatibility is checked.

1. Install Liftoff through Steam and run it normally once. Install **Python 3** if needed and ensure **Rosetta** is available on Apple Silicon. Keep Steam open and signed in, then quit Liftoff.
2. Download this repository with **Code → Download ZIP** and unzip it, or clone it with Git.
3. Open **liftoff_mod** and double-click **Setup Geometry Lab.command**. It downloads the build tools and mod loader, builds the mod, and runs its tests. Wait for **“Geometry Lab built and staged.”**
4. Connect your radio. For the RadioMaster Pocket, use the **top USB-C port** and choose **USB Joystick**. Configure/calibrate it in Liftoff.
5. Double-click **Launch Liftoff Geometry Lab.command**. Enter **Single Player → Free Flight**, choose a four-motor ZetaFlight drone, and stop on the ground.
6. Open the editor with **F8**, then click **Apply GoPro Drone / edited geometry**. This applies the geometry and controller compensation together. **Reapply after each drone reset.**

Use the mod launcher for subsequent sessions. To fly without the mod, quit and launch Liftoff normally through Steam.

**[Full installation guide, prerequisites, custom game locations, updates, and troubleshooting →](liftoff_mod/README.md#install-on-another-computer)**

The first setup needs an internet connection. The repository does not include Liftoff, downloaded dependencies, or a prebuilt mod. Windows still needs a loader/launcher and testing against its own game files.

## Project folders

- **[liftoff_mod](liftoff_mod/README.md)** — the GoPro Drone deadcat geometry mod for Liftoff, including motor positions, mass, CG, inertia, and controller compensation. See its validation status and limitations before drawing design conclusions.
- **[flight_sim](flight_sim/README.md)** — the earlier standalone native Mac simulator with RadioMaster Pocket USB input, Betaflight, and a 3D flying field.
- **[matlab_sim](matlab_sim/README.md)** — the original MATLAB geometry analysis and scripted 6DOF simulation. In MATLAB, change the current folder to `matlab_sim`, then run `run_all_simulations`.

## Earlier standalone simulator

This is separate from the Liftoff mod. To install it on another Mac, run `flight_sim/tools/setup.sh` from the repository folder, then open **flight_sim/Launch Flight Sim.command**. Git excludes its downloaded Godot and Betaflight dependencies, so each fresh checkout needs setup.

Choose **GoPro Drone** (the original deadcat model) or **DJI FPV** in the standalone simulator's flight window. DJI FPV is an explicitly approximate aircraft model using published dimensions and mass, with estimated dynamics and Betaflight Acro controls. See [aircraft model sources and assumptions](flight_sim/docs/AIRCRAFT_MODELS.md).

To import later edits to the original MATLAB inputs, run `python3 flight_sim/tools/import_config.py`. To import GoPro Drone thrust measurements, see the [flight simulator README](flight_sim/README.md#measured-thrust-data).

📘 Plugin for GRBLHAL — Internal Keyway Cycle (M800)

🔧 Description
Plugin Name: Lathe Internal Keyway Cycle
G‑code: M800  
Purpose: Automatic machining cycle for internal longitudinal keyways
Features:

Multi‑pass radial cutting

Full‑length Z strokes for chip evacuation

Sag compensation for internal bores

Safe retract logic

Optional return to start

Full validation of parameters

No modifications to GRBLHAL core required

🧩 1. Installing the plugin
Copy the folder keyway/ into the Src/ directory of your GRBLHAL project:

Codice
Src/
    keyway/
        keyway.c
        CMakeLists.txt
Placing the plugin inside Src/ ensures that PlatformIO automatically compiles it without additional configuration.

🧩 2. Update platformio.ini
In the environment you are using (e.g., blackpill_f411ce_alt2), add:

✔ Include path
Codice
-I Src/keyway
✔ Source filter
Codice
build_src_filter = +<> +<Src/keyway/.c>
✔ Example
Codice
[env:blackpill_f411ce_alt2]
build_flags = ${common.build_flags} -I Src/keyway
build_src_filter = +<> +<Src/keyway/.c>

🧩 3. Planner patch — NOT REQUIRED
Older versions of the plugin required a patch to force G0 moves.
The current version does not require any modification to GRBLHAL.

The plugin uses:

plan_data_init()

plan_line_data_t.condition.rapid_motion

mc_line()

protocol_buffer_synchronize()

All motion is handled using standard GRBLHAL mechanisms.

✔ Fully compatible with upstream GRBLHAL

🧩 4. Using the M800 command
Syntax

M800 D<depth> Q<length> S<tool width> P<step> R<retract> [L<reps>] [H<return>]

Parameters

Word	Meaning	Notes

D	Final X depth	> 0

Q	Keyway length in Z	> 0 (cut in −Z)

S	Tool width	used for sag compensation

P	Depth step per pass	P ≤ D

R	Z retract	> 0

L	Repetitions per depth	integer ≥ 1 (default = 1)

H	Return to start	H1 = yes (default), H0 = no

Example

G90
G21

M5

G0 X10 Z10

F1000

M800 D2 Q10 S8 P0.5 R2 L1 H1

M30

🧩 5. Compatibility
The M800 Keyway plugin has been tested on:

WeAct Blackpill F411CE (STM32F411CEU6)

Other GRBLHAL boards may be compatible but have not yet been tested.

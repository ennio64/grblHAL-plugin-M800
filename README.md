📘 Plugin for GRBLHAL

🔧 Description

Plugin Name: Lathe Keyway Cycle

G-code: M800

Description: Parametric keyway machining cycle for lathes

Features: Multi-pass, safe return logic, validation, universal geometry.


🧩 1. Copy the plugin
Copy the folder:
keyway/
into the Src/ directory of your GRBLHAL project, for example:


    Src/

        keyway/
    
            keyway_plugin.c
        
            keyway_plugin.h
        

platformio.ini
Placing the plugin inside Src/ ensures that PlatformIO automatically compiles it without additional configuration.


🧩 2. Update platformio.ini
In the environment you are using (e.g., blackpill_f411ce_alt2), add:

✔ Add the plugin directory to the include paths
ini

-I Src/keyway

✔ Add the plugin directory to the source filter
ini

build_src_filter = +<*> +<Src/keyway/*.c>

✔ Example
ini
[env:blackpill_f411ce_alt2]

build_flags = ${common.build_flags}
  -I Src/keyway

build_src_filter = +<*> +<Src/keyway/*.c>


🧩 3. Required patch to the GRBLHAL planner
The plugin requires a small extension to the planner to allow forced G0 moves.

✔ In planner.h, inside plan_line_data_t, add:
c

bool plugin_force_rapid; // <--- ADDED FOR PATCH: plugin override to force G0 in M800

✔ In planner.c, inside the block construction logic, add:
c

// --- PATCH: plugin override to force G0 in M800 ---

if(pl_data->plugin_force_rapid)    block->condition.rapid_motion = On;
    
// --- END PATCH ---

This is the only required patch.

The repository already includes the pre‑patched GRBLHAL (Version: 20251207) source files for convenience.
  
These files contain the patch above already applied, allowing immediate compilation without manual modifications.


🧩 4. Using the M800 command
Syntax
M800 D<depth X> L<length Z> P<step X> R<retract Z> F<feed> [N<repetitions>] [H<return>]

Example

G90
G21

M5

G0 X10 Z10

M800 D2 L10 P0.5 R2 F1000 N1 H1


✔ Compatibility
The M800 Keyway plugin has been tested and verified on:

WeAct Blackpill F411CE (STM32F411CEU6)

Other GRBLHAL boards may be compatible, but have not yet been tested.

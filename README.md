📘 Plugin for GRBLHAL — Internal Keyway Cycle (M800)

🔧 Description
Plugin Name: Lathe Internal Keyway Cycle

G‑code: M800  

Purpose: Automatic machining cycle for internal longitudinal keyways

Features:

Radial depth stepping

Axial cutting strokes

Sag compensation for correct geometry and collision avoidance

Safe retract logic

Optional return to start

Full parameter validation


🧩 1. Installation

A. Plugin Placement

Copy the keyway/ folder to the ROOT of your GRBLHAL project (same level as grbl/, boards/, etc.):


    your_grblhal_project/

    ├── keyway/           ← Copy this folder here
    
    │   └── keyway.c

    ├── grbl/

    ├── boards/

    ├── Inc/

    ├── Src/

    └── platformio.ini

⚠️ It must be at the same level as the grbl folder.

B. PlatformIO Configuration

Add these lines to your platformio.ini:

In the [common] section:

[common]

lib_deps =

      ... other directories ...
      
    keyway           ← Add this line
    
In your specific environment (e.g., blackpill_f411ce_alt2):


[env:blackpill_f411ce_alt2]

build_flags = ${common.build_flags}

     ... other flags ...
    
    -D M800_ENABLE=1    # Enable the plugin
    
    -D M800_DEBUG=1     # Enable debug output (optional)
    

C. Update plugins_init.h

Add this code to Inc/plugins_init.h (in the "Third party plugin definitions" section):

    // ... existing code ...
    
    // Third party plugin definitions.
    
    #if M800_ENABLE
        extern void keyway_init (void);
        keyway_init();
    #endif

    // ... rest of the code ...


🧩 3. Patch — NOT REQUIRED

Older versions of the plugin required a patch planner to force G0 moves.

The current version does not require any modification to GRBLHAL.

The plugin uses:

plan_data_init()

plan_line_data_t.condition.rapid_motion

mc_line()

protocol_buffer_synchronize()

All motion is handled using standard GRBLHAL mechanisms.

Fully compatible with upstream GRBLHAL

🧩 4. Using the M800 command Syntax

    M800 D<depth> Q<length> S<tool width> P<step> R<retract> [L<reps>] [H<return>]

Parameters

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

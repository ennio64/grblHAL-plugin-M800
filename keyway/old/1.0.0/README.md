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





&nbsp;   your\_grblhal\_project/



&nbsp;   ├── keyway/           ← Copy this folder here

&nbsp;   

&nbsp;   │   └── keyway.c



&nbsp;   ├── grbl/



&nbsp;   ├── boards/



&nbsp;   ├── Inc/



&nbsp;   ├── Src/



&nbsp;   └── platformio.ini



⚠️ It must be at the same level as the grbl folder.



B. PlatformIO Configuration



Add these lines to your platformio.ini:



In the \[common] section:



\[common]



lib\_deps =



&nbsp;     ... other directories ...

&nbsp;     

&nbsp;   keyway           ← Add this line

&nbsp;   

In your specific environment (e.g., blackpill\_f411ce\_alt2):





\[env:blackpill\_f411ce\_alt2]



build\_flags = ${common.build\_flags}



&nbsp;    ... other flags ...

&nbsp;   

&nbsp;   -D M800\_ENABLE=1    # Enable the plugin

&nbsp;   

&nbsp;   -D M800\_DEBUG=1     # Enable debug output (optional)

&nbsp;   



C. Update plugins\_init.h



Add this code to Inc/plugins\_init.h (in the "Third party plugin definitions" section):



&nbsp;   // ... existing code ...

&nbsp;   

&nbsp;   // Third party plugin definitions.

&nbsp;   

&nbsp;   #if M800\_ENABLE

&nbsp;       extern void keyway\_init (void);

&nbsp;       keyway\_init();

&nbsp;   #endif



&nbsp;   // ... rest of the code ...





🧩 3. Patch — NOT REQUIRED



Older versions of the plugin required a patch planner to force G0 moves.



The current version does not require any modification to GRBLHAL.



The plugin uses:



plan\_data\_init()



plan\_line\_data\_t.condition.rapid\_motion



mc\_line()



protocol\_buffer\_synchronize()



All motion is handled using standard GRBLHAL mechanisms.



Fully compatible with upstream GRBLHAL



🧩 4. Using the M800 command Syntax



&nbsp;   M800 D<depth> Q<length> S<tool width> P<step> R<retract> \[L<reps>] \[H<return>]



Parameters



D	Final X depth	> 0



Q	Keyway length in Z	> 0 (cut in −Z)



S	Tool width	used for sag compensation



P	Depth step per pass	P ≤ D



R	Z retract	> 0



L	Repetitions per depth	integer ≥ 1 (default = 1)



H	Return to start	H1 = yes (default), H0 = no



&nbsp;   Example



&nbsp;   G90

&nbsp;   G21

&nbsp;   M5

&nbsp;   G0 X10 Z10

&nbsp;   F1000

&nbsp;   M800 D2 Q10 S8 P0.5 R2 L1 H1

&nbsp;   M30



🧩 5. Compatibility

The M800 Keyway plugin has been tested on:



WeAct Blackpill F411CE (STM32F411CEU6)



Other GRBLHAL boards may be compatible but have not yet been tested.




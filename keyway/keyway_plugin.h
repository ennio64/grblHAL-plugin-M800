// ============================================================================
//  M800 – Longitudinal Keyway Cutting Cycle for Lathe
// ============================================================================
//
//  Description:
//      Executes an automatic cycle for machining longitudinal keyways on a lathe,
//      with progressive plunges in X, passes in Z, and rapid retracts.
//      The cycle uses G1 moves for cutting passes and G0 moves for retracts,
//      ensuring stable, repeatable, and safe behavior.
//      The spindle must be stopped.
//
//  Official syntax:
//
//      M800 D<final X depth>
//           L<keyway length in Z>
//           P<X step per pass>
//           R<Z retract>
//           F<feed>
//           [N<repetitions per depth level>]   (optional)
//           [H<final return>]                  (optional)
//
//  Parameters:
//
//      D   Final depth in X (POSITIVE value).
//      L   Keyway length in Z (POSITIVE value).
//      P   Depth increment per pass (POSITIVE value).
//      R   Z retract distance before each plunge (POSITIVE value).
//      F   Feed rate for G1 cutting passes.
//
//      N   Number of repetitions at each depth level.
//          Default = N1.
//
//      H   Return to the initial position at the end of the cycle.
//          H1 = return (default)
//          H0 = do not return
//
// ----------------------------------------------------------------------------
//  COMPLETE PROGRAM EXAMPLE
// ----------------------------------------------------------------------------
//
//  G90                 ; absolute mode
//  G21                 ; units in millimeters
//  M5                  ; stop spindle (safety)
//
//  G0 X10 Z10          ; initial positioning
//
//  M800 D2 L10 P0.5 R2 F1000 N1 H1  ; full keyway cycle with final return
//
//  M30                 ; end of program
//
// ----------------------------------------------------------------------------
//  TECHNICAL NOTES – Handling sys.position and planner patch in GRBLHAL
// ----------------------------------------------------------------------------
//
//  In GRBLHAL, the planner and G-code state always operate in millimeters
//  (gc_state.position). However, sys.position[] is internally updated by the
//  stepper engine in STEPS. This is expected behavior and not an error.
//
//  The M800 plugin converts sys.position[] to millimeters by dividing by
//  steps_per_mm, ensuring an initial position consistent with the planner’s
//  coordinate system.
//
//  Additionally, to guarantee correct cycle behavior (forced G0 returns,
//  mandatory rapid moves, elimination of G0/G1 ambiguity), a small patch must
//  be applied to planner.h and planner.c:
//
//      • Add the field `plugin_force_rapid` to plan_line_data_t
//      • Modify the G0/G1 selection logic to honor this flag
//
//  This extension allows the M800 plugin to force rapid G0 moves regardless of
//  the planner’s internal conditions, while maintaining full compatibility with
//  GRBLHAL and without altering the standard behavior of the firmware for other
//  commands.
// ----------------------------------------------------------------------------

#ifndef KEYWAY_PLUGIN_H
#define KEYWAY_PLUGIN_H

#include "grbl/hal.h"
#include "grbl/gcode.h"

void my_plugin_init(void);

#endif // KEYWAY_PLUGIN_H

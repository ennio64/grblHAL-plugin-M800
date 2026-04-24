// ============================================================================
//  M800 – Internal Longitudinal Keyway Cutting Cycle for Lathe
// ============================================================================
//
//  Description:
//      Executes an automatic cycle for machining *internal* longitudinal keyways
//      such as those found inside pulleys, hubs, bushings, and sleeves.
//      These operations require the tool to travel the *entire Z length* of the
//      keyway to fully evacuate chips, because the tool works inside a closed
//      bore and cannot discharge chips radially.
//
//      The cycle is designed for internal slotting tools, broach‑like cutters,
//      and single‑point internal keyway knives mounted radially on the lathe.
//      These tools cut by plunging in X and stroking in Z.
//
//      The cycle performs:
//          • Initial positioning at the current X/Z coordinates
//          • A first "dry" pass with zero penetration (safety pass)
//          • Progressive depth increments in X (P per pass)
//          • Full‑length cutting strokes in Z (length Q)
//          • Rapid retracts in X and Z using G0
//          • Optional return to the initial position (H)
//
//      The spindle must be stopped before executing M800.
//
// ----------------------------------------------------------------------------
//  SAG COMPENSATION (Geometric Correction)
// ----------------------------------------------------------------------------
//
//      When cutting inside a bore, the tool has a finite width (S).
//      The cutting edge does *not* lie exactly on the bore radius, but is
//      geometrically offset inward. This offset is known as **sag**.
//
//      Sag is the radial difference between:
//          • the bore radius
//          • the distance from the bore center to the tool's cutting edge
//
//      Formula:
//          sag = Rbore – sqrt( Rbore² – (S/2)² )
//
//      The cycle automatically:
//          • shifts the starting X position inward by "sag"
//          • increases the commanded depth D by "sag"
//          • ensures the final X position corresponds to the *true* depth
//
//      This guarantees:
//          • physically correct geometry
//          • correct keyway width at full depth
//          • no over‑cutting or under‑cutting due to tool width
//          • no collision between the tool flanks and the bore walls
//
// ----------------------------------------------------------------------------
//  Official syntax
// ----------------------------------------------------------------------------
//
//      M800 D<final X depth>
//           Q<keyway length in Z>
//           S<tool width>
//           P<X step per pass>
//           R<Z retract>
//           [L<repetitions per depth level>]   (optional)
//           [H<final return>]                  (optional)
//
//  Parameters:
//
//      D   Final depth in X (POSITIVE value).
//          Eg. D2 → final X = X_start + 2 mm
//
//      Q   Keyway length in Z (POSITIVE value).
//          Internally the cycle moves in negative Z by -Q.
//
//      S   Tool width (POSITIVE value).
//          Used for sag compensation.
//
//      P   Depth increment per pass (POSITIVE value).
//          Must satisfy P ≤ D.
//
//      R   Z retract distance before each plunge (POSITIVE value).
//
//      L   Number of repetitions at each depth level (integer ≥ 1).
//          Used when multiple cutting strokes are required at the same depth.
//          Default = L1.
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
//  F1000               ; feed rate for cutting passes (used by M800)
//
//  M800 D2 Q10 S2 P0.1 R2 L1 H1   ; full internal keyway cycle
//
//  M30                 ; end of program
//
// ----------------------------------------------------------------------------
//  TECHNICAL NOTES – Motion Planning and Synchronization
// ----------------------------------------------------------------------------
//
//  The M800 cycle uses only standard GRBLHAL mechanisms:
//
//      • plan_data_init() to initialize motion structures
//      • mc_line() for both G0 and G1 moves
//      • plan_line_data_t.condition.rapid_motion to select rapid/feed motion
//      • gc_state.feed_rate for cutting feed (F-word not trapped locally)
//      • protocol_buffer_synchronize() to ensure all moves are completed
//
//  No modifications to the GRBLHAL core are required.
//
//  "M800 CYCLE END" is printed only after the planner buffer is fully empty,
//  guaranteeing that the cycle has physically completed.
//
// ----------------------------------------------------------------------------
//  DEBUG MODE (M800_DEBUG)
// ----------------------------------------------------------------------------
//
//  When M800_DEBUG = 1, the cycle prints:
//
//      • Geometry and sag compensation
//      • Pre‑positioning coordinates
//      • First safety pass
//      • Each depth pass with pass/rep counters
//      • Final return coordinates
//
//  When M800_DEBUG = 0, only:
//
//      • M800 CYCLE START
//      • M800 CYCLE END
//
//  are printed.
//
// ----------------------------------------------------------------------------
//  SAFETY NOTES
// ----------------------------------------------------------------------------
//
//      • The spindle must be stopped (M5).
//      • Ensure the tool is aligned radially.
//      • Ensure R is sufficient to clear the workpiece.
//      • Ensure P ≤ D.
//      • Ensure Q > 0.
//      • Ensure S > 0.
//      • Ensure no other axes are commanded during the cycle.
//      • The cycle is fully deterministic and repeatable.
//
// ----------------------------------------------------------------------------

#include "keyway.h"

#ifdef M800_ENABLE

#ifndef M800_DEBUG
#define M800_DEBUG 1   // 1 = debug ON, 0 = debug OFF
#endif

#if M800_DEBUG
#define M800_LOG(...) do { snprintf(dbg, sizeof(dbg), __VA_ARGS__); hal.stream.write(dbg); } while(0)
#else
#define M800_LOG(...)
#endif

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/motion_control.h"
#include "grbl/state_machine.h"
#include "grbl/nuts_bolts.h"
#include "grbl/gcode.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define M800_Internal 800

extern stepper_t st;
static user_mcode_ptrs_t user_mcode_prev;


// -----------------------------------------------------------------------------
// HELPER: GET CURRENT MACHINE POSITION IN MM/DEGREES
// -----------------------------------------------------------------------------

static void m800_get_current_pos(float pos[N_AXIS])
{
    for (uint_fast8_t axis = 0; axis < N_AXIS; axis++) {
        bool axis_present = (settings.axis[axis].steps_per_mm > 0.0f) ||
                            (settings.axis[axis].max_rate > 0.0f);
        
        if (axis_present) {
            pos[axis] = sys.position[axis] / settings.axis[axis].steps_per_mm;
        } else {
            pos[axis] = 0.0f;
        }
    }
}


// -----------------------------------------------------------------------------
// HELPER: COPY POSITION FROM SOURCE TO TARGET (ALL AXES)
// -----------------------------------------------------------------------------

static void m800_copy_pos(float target[N_AXIS], const float source[N_AXIS])
{
    for (uint_fast8_t axis = 0; axis < N_AXIS; axis++) {
        target[axis] = source[axis];
    }
}


// -----------------------------------------------------------------------------
// CHECK
// -----------------------------------------------------------------------------

static user_mcode_type_t m800_check(user_mcode_t mcode)
{
    if(mcode == M800_Internal)
        return UserMCode_NoValueWords;

    return user_mcode_prev.check ?
           user_mcode_prev.check(mcode) :
           UserMCode_Unsupported;
}


// -----------------------------------------------------------------------------
// VALIDATE
// -----------------------------------------------------------------------------

static status_code_t m800_validate(parser_block_t *gc_block)
{
    if(gc_block->user_mcode != M800_Internal)
        return user_mcode_prev.validate ?
               user_mcode_prev.validate(gc_block) :
               Status_Unhandled;

    if(gc_block->values.d <= 0.0f) return Status_InvalidStatement;
    if(gc_block->values.q <= 0.0f) return Status_InvalidStatement;
    if(gc_block->values.s <= 0.0f) return Status_InvalidStatement;
    if(gc_block->values.p <= 0.0f) return Status_InvalidStatement;
    if(gc_block->values.r <= 0.0f) return Status_InvalidStatement;

    if(gc_block->values.p > gc_block->values.d)
        return Status_InvalidStatement;

    if(gc_block->words.l && gc_block->values.l < 1)
        return Status_InvalidStatement;

    if(gc_block->words.h)
        if(gc_block->values.h != 0.0f && gc_block->values.h != 1.0f)
            return Status_InvalidStatement;

    if(gc_state.feed_rate <= 0.0f)
        return Status_InvalidStatement;

    gc_block->words.d = Off;
    gc_block->words.q = Off;
    gc_block->words.s = Off;
    gc_block->words.p = Off;
    gc_block->words.r = Off;
    gc_block->words.l = Off;
    gc_block->words.h = Off;

    return Status_OK;
}


// -----------------------------------------------------------------------------
// EXECUTE
// -----------------------------------------------------------------------------

static void m800_execute(uint_fast16_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode != M800_Internal) {
        if(user_mcode_prev.execute)
            user_mcode_prev.execute(state, gc_block);
        return;
    }

    char dbg[128];

    // -------------------------------------------------------------------------
    // GET STARTING POSITIONS (ALL AXES)
    // -------------------------------------------------------------------------
    float start_pos[N_AXIS];
    float X_start = 0.0f, Z_start = 0.0f;

    m800_get_current_pos(start_pos);
    X_start = start_pos[X_AXIS];
    Z_start = start_pos[Z_AXIS];

    // -------------------------------------------------------------------------
    // PARAMETERS
    // -------------------------------------------------------------------------
    float D = gc_block->values.d;
    float Q = -gc_block->values.q;           // Negative Z direction
    float W =  gc_block->values.s;
    float P =  gc_block->values.p;
    float R =  gc_block->values.r;
    int   Lreps = gc_block->words.l ? (int)gc_block->values.l : 1;
    bool return_home = (gc_block->values.h == 1.0f);

    protocol_buffer_synchronize();

    // ALWAYS ON
    hal.stream.write("M800 CYCLE START\r\n");

    // DEBUG
    M800_LOG("M800 GEOMETRY: X0=%.3f Z0=%.3f Q=%.3f W=%.3f Feed=%.3f\r\n",
             X_start, Z_start, Q, W, gc_state.feed_rate);

    // -------------------------------------------------------------------------
    // SAG COMPENSATION
    // -------------------------------------------------------------------------
    float Rbore = X_start;
    float Cslot = W;
    float halfC = Cslot * 0.5f;

    if(halfC > Rbore) {
        report_message("M800: Slot width exceeds bore diameter.", Message_Warning);
        hal.stream.write("M800 CYCLE END\r\n");
        return;
    }

    float d_center = sqrtf((Rbore * Rbore) - (halfC * halfC));
    float sag = Rbore - d_center;

    float X_new_start = X_start - sag;
    float Dcorr = D + sag;
    float X_final = X_new_start + Dcorr;

    M800_LOG("M800 SAG: R=%.3f C=%.3f sag=%.3f X_new_start=%.3f Dcorr=%.3f Xfinal=%.3f\r\n",
             Rbore, Cslot, sag, X_new_start, Dcorr, X_final);

    M800_LOG("M800 AXIS MASK: XZ USED | ALL OTHER AXES PRESERVED (Y, A, B, C, etc.)\r\n");

    // -------------------------------------------------------------------------
    // PLAN DATA INITIALIZATION
    // -------------------------------------------------------------------------
    plan_line_data_t plan_g0;
    plan_line_data_t plan_g1;

    plan_data_init(&plan_g0);
    plan_data_init(&plan_g1);

    plan_g0.condition.rapid_motion = On;
    plan_g0.spindle = *gc_state.spindle;

    plan_g1.condition.rapid_motion = Off;
    plan_g1.feed_rate = gc_state.feed_rate;
    plan_g1.spindle = *gc_state.spindle;

    // -------------------------------------------------------------------------
    // TRACK LAST COMMANDED POSITION (CRITICAL FOR COORDINATE COHERENCE)
    // -------------------------------------------------------------------------
    float last_commanded[N_AXIS];
    m800_copy_pos(last_commanded, start_pos);

    float target[N_AXIS];

    // -------------------------------------------------------------------------
    // PRE-POSITIONING (G0 to sag-compensated X + retract Z)
    // -------------------------------------------------------------------------
    m800_copy_pos(target, last_commanded);
    target[X_AXIS] = X_new_start;
    target[Z_AXIS] = Z_start + R;

    M800_LOG("M800 G0 SAG POS: X=%.3f Z=%.3f\r\n",
             target[X_AXIS], target[Z_AXIS]);

    mc_line(target, &plan_g0);
    m800_copy_pos(last_commanded, target);

    // -------------------------------------------------------------------------
    // FIRST PASS (ZERO PENETRATION - SAFETY PASS)
    // -------------------------------------------------------------------------
    for(int rep = 0; rep < Lreps; rep++) {

        // G0 SAFE (already at correct position, but included for clarity)
        m800_copy_pos(target, last_commanded);
        target[X_AXIS] = X_new_start;
        target[Z_AXIS] = Z_start + R;
        M800_LOG("M800 G0 SAFE (FIRST): X=%.3f Z=%.3f (pass=0 rep=%d)\r\n",
                 target[X_AXIS], target[Z_AXIS], rep+1);
        mc_line(target, &plan_g0);
        m800_copy_pos(last_commanded, target);

        // G1 DEPTH (plunge in X only, Z unchanged)
        m800_copy_pos(target, last_commanded);
        target[X_AXIS] = X_new_start;
        // Z remains at Z_start + R
        M800_LOG("M800 G1 DEPTH (FIRST): X=%.3f Z=%.3f (pass=0 rep=%d)\r\n",
                 target[X_AXIS], target[Z_AXIS], rep+1);
        mc_line(target, &plan_g1);
        m800_copy_pos(last_commanded, target);

        // G1 LENGTH (cut in Z only, X unchanged)
        m800_copy_pos(target, last_commanded);
        target[Z_AXIS] = Z_start + Q;
        M800_LOG("M800 G1 LENGTH (FIRST): X=%.3f Z=%.3f (pass=0 rep=%d)\r\n",
                 target[X_AXIS], target[Z_AXIS], rep+1);
        mc_line(target, &plan_g1);
        m800_copy_pos(last_commanded, target);

        // G0 BACKX (retract in X only, Z unchanged)
        m800_copy_pos(target, last_commanded);
        target[X_AXIS] = X_new_start;
        M800_LOG("M800 G0 BACKX (FIRST): X=%.3f Z=%.3f (pass=0 rep=%d)\r\n",
                 target[X_AXIS], target[Z_AXIS], rep+1);
        mc_line(target, &plan_g0);
        m800_copy_pos(last_commanded, target);

        // G0 BACKZ (retract in Z only, X unchanged)
        m800_copy_pos(target, last_commanded);
        target[Z_AXIS] = Z_start + R;
        M800_LOG("M800 G0 BACKZ (FIRST): X=%.3f Z=%.3f (pass=0 rep=%d)\r\n",
                 target[X_AXIS], target[Z_AXIS], rep+1);
        mc_line(target, &plan_g0);
        m800_copy_pos(last_commanded, target);
    }

    // -------------------------------------------------------------------------
    // RADIAL PASSES (PROGRESSIVE DEPTH)
    // -------------------------------------------------------------------------
    int passes = (int)ceilf(Dcorr / P);

    M800_LOG("M800 PASSES=%d L=%d\r\n", passes, Lreps);

    for(int pass = 1; pass <= passes; pass++) {

        float X_target = X_new_start + pass * P;
        if(X_target > X_final)
            X_target = X_final;

        for(int rep = 0; rep < Lreps; rep++) {

            // G0 SAFE (position at Z_start + R, X at X_new_start)
            m800_copy_pos(target, last_commanded);
            target[X_AXIS] = X_new_start;
            target[Z_AXIS] = Z_start + R;
            M800_LOG("M800 G0 SAFE:   X=%.3f Z=%.3f (pass=%d rep=%d)\r\n",
                     target[X_AXIS], target[Z_AXIS], pass, rep+1);
            mc_line(target, &plan_g0);
            m800_copy_pos(last_commanded, target);

            // G1 DEPTH (plunge to new X depth, Z unchanged)
            m800_copy_pos(target, last_commanded);
            target[X_AXIS] = X_target;
            M800_LOG("M800 G1 DEPTH:  X=%.3f Z=%.3f (pass=%d rep=%d)\r\n",
                     target[X_AXIS], target[Z_AXIS], pass, rep+1);
            mc_line(target, &plan_g1);
            m800_copy_pos(last_commanded, target);

            // G1 LENGTH (cut full Z length, X unchanged)
            m800_copy_pos(target, last_commanded);
            target[Z_AXIS] = Z_start + Q;
            M800_LOG("M800 G1 LENGTH: X=%.3f Z=%.3f (pass=%d rep=%d)\r\n",
                     target[X_AXIS], target[Z_AXIS], pass, rep+1);
            mc_line(target, &plan_g1);
            m800_copy_pos(last_commanded, target);

            // G0 BACKX (retract X to safe X, Z unchanged)
            m800_copy_pos(target, last_commanded);
            target[X_AXIS] = X_new_start;
            M800_LOG("M800 G0 BACKX:  X=%.3f Z=%.3f (pass=%d rep=%d)\r\n",
                     target[X_AXIS], target[Z_AXIS], pass, rep+1);
            mc_line(target, &plan_g0);
            m800_copy_pos(last_commanded, target);

            // G0 BACKZ (retract Z to safe Z, X unchanged)
            m800_copy_pos(target, last_commanded);
            target[Z_AXIS] = Z_start + R;
            M800_LOG("M800 G0 BACKZ:  X=%.3f Z=%.3f (pass=%d rep=%d)\r\n",
                     target[X_AXIS], target[Z_AXIS], pass, rep+1);
            mc_line(target, &plan_g0);
            m800_copy_pos(last_commanded, target);
        }
    }

    // -------------------------------------------------------------------------
    // FINAL RETURN
    // -------------------------------------------------------------------------
    plan_data_init(&plan_g0);
    plan_g0.condition.rapid_motion = On;
    plan_g0.spindle = *gc_state.spindle;

    m800_copy_pos(target, last_commanded);
    if(return_home) {
        target[X_AXIS] = X_start;
        target[Z_AXIS] = Z_start;
    } else {
        target[X_AXIS] = X_new_start;
        target[Z_AXIS] = Z_start + R;
    }

    M800_LOG("M800 RETURN: X=%.3f Z=%.3f\r\n",
             target[X_AXIS], target[Z_AXIS]);

    mc_line(target, &plan_g0);
    m800_copy_pos(last_commanded, target);

    protocol_buffer_synchronize();

    // ALWAYS ON
    hal.stream.write("M800 CYCLE END\r\n");
}


// -----------------------------------------------------------------------------
// INIT
// -----------------------------------------------------------------------------

void keyway_init(void)
{
    memcpy(&user_mcode_prev, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check    = m800_check;
    grbl.user_mcode.validate = m800_validate;
    grbl.user_mcode.execute  = m800_execute;
}

#endif // M800_ENABLE
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/motion_control.h"
#include "grbl/state_machine.h"
#include "grbl/nuts_bolts.h"
#include "grbl/gcode.h"

#include <math.h>
#include <stdio.h>

#define M800_Internal 800

extern stepper_t st;
static user_mcode_ptrs_t user_mcode_prev;


// -----------------------------------------------------------------------------
//  CHECK
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
//  VALIDATE
// -----------------------------------------------------------------------------

static status_code_t m800_validate(parser_block_t *gc_block)
{
    if(gc_block->user_mcode != M800_Internal)
        return user_mcode_prev.validate ?
               user_mcode_prev.validate(gc_block) :
               Status_Unhandled;

    if(gc_block->values.d <= 0.0f) {
        report_message("M800: D must be > 0.", Message_Warning);
        return Status_InvalidStatement;
    }

    if(gc_block->values.l <= 0.0f) {
        report_message("M800: L must be > 0.", Message_Warning);
        return Status_InvalidStatement;
    }

    if(gc_block->values.p <= 0.0f) {
        report_message("M800: P must be > 0.", Message_Warning);
        return Status_InvalidStatement;
    }

    if(gc_block->values.r <= 0.0f) {
        report_message("M800: R must be > 0.", Message_Warning);
        return Status_InvalidStatement;
    }

    if(gc_block->values.f <= 0.0f) {
        report_message("M800: F must be > 0.", Message_Warning);
        return Status_InvalidStatement;
    }

    if(gc_block->values.p > gc_block->values.d) {
        report_message("M800: P cannot be greater than D.", Message_Warning);
        return Status_InvalidStatement;
    }

    if(gc_block->words.n && gc_block->values.n < 1) {
        report_message("M800: N must be >= 1.", Message_Warning);
        return Status_InvalidStatement;
    }

    // --- VALIDAZIONE PARAMETRO H ---
    // H0 = non tornare alla posizione iniziale
    // H1 = tornare alla posizione iniziale (default)
    if(gc_block->words.h) {
        if(gc_block->values.h != 0.0f && gc_block->values.h != 1.0f) {
            report_message("M800: H must be 0 or 1.", Message_Warning);
            return Status_InvalidStatement;
        }
    }

    // --- Reset dei flag words ---
    gc_block->words.d = Off;
    gc_block->words.l = Off;
    gc_block->words.p = Off;
    gc_block->words.r = Off;
    gc_block->words.f = Off;
    gc_block->words.n = Off;
    gc_block->words.h = Off;

    return Status_OK;
}


// -----------------------------------------------------------------------------
//  EXECUTE
// -----------------------------------------------------------------------------

static void m800_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    char dbg[96];

    if(gc_block->user_mcode != M800_Internal) {
        if(user_mcode_prev.execute)
            user_mcode_prev.execute(state, gc_block);
        return;
    }

    // -------------------------------------------------------------------------
    //  Parametri M800
    // -------------------------------------------------------------------------
    float D = gc_block->values.d;                      // profondità totale (negativa sul pezzo)
    float L = -gc_block->values.l;                     // lunghezza cava (Z negativo)
    float P = -gc_block->values.p;                     // incremento per passata
    float R = gc_block->values.r;                      // distanza di sicurezza in Z
    float F = gc_block->values.f;                      // feed
    int   N = gc_block->words.n ? (int)gc_block->values.n : 1;

    float H = gc_block->values.h;                      // H0 = no return, H1 = return
    bool return_home = (H == 0.0f ? false : true);

    // -------------------------------------------------------------------------
    //  Sincronizza con il planner
    // -------------------------------------------------------------------------
    protocol_buffer_synchronize();

    // Lettura posizione macchina (in passi → mm)
    float X_raw = sys.position[X_AXIS];
    float Z_raw = sys.position[Z_AXIS];

    float X_start = X_raw / settings.axis[X_AXIS].steps_per_mm;
    float Z_start = Z_raw / settings.axis[Z_AXIS].steps_per_mm;

    snprintf(dbg, sizeof(dbg),
             "M800 START: rawX=%.3f rawZ=%.3f  X0=%.3f Z0=%.3f  gcX=%.3f gcZ=%.3f H=%.1f\r\n",
             X_raw, Z_raw, X_start, Z_start,
             gc_state.position[X_AXIS], gc_state.position[Z_AXIS], H);
    hal.stream.write(dbg);

    hal.stream.write("M800 CYCLE BEGIN\r\n");

    // -------------------------------------------------------------------------
    //  Dati movimento
    // -------------------------------------------------------------------------
    plan_line_data_t plan_g1 = (plan_line_data_t){0};
    plan_g1.feed_rate = F;
    plan_g1.plugin_force_rapid = false;
    plan_g1.condition.rapid_motion = Off;
    plan_g1.spindle = *gc_state.spindle;

    plan_line_data_t plan_g0 = (plan_line_data_t){0};
    plan_g0.plugin_force_rapid = true;
    plan_g0.condition.rapid_motion = On;
    plan_g0.spindle = *gc_state.spindle;

    // Target iniziale = sys.position (in passi)
    float target[N_AXIS];
    memcpy(target, sys.position, sizeof(target));

    float total_depth = -D;    // D negativa → profondità positiva
    int passes = (int)(fabsf(total_depth / P) + 0.0001f);

    snprintf(dbg, sizeof(dbg), "M800 PASSES=%d N=%d\r\n", passes, N);
    hal.stream.write(dbg);

    float current_depth = 0.0f;

    // -------------------------------------------------------------------------
    //  Passate
    // -------------------------------------------------------------------------
    for(int pass = 0; pass < passes; pass++) {

        current_depth += P;

        for(int rep = 0; rep < N; rep++) {

            // ------------------------------------------------------------
            // 1) SAFE: rapid move to safe height (always print comment)
            // ------------------------------------------------------------
            float safeX = X_start;
            float safeZ = Z_start + R;

            snprintf(dbg, sizeof(dbg),
                     "M800 G0 SAFE:   X=%.3f Z=%.3f (pass=%d rep=%d)\r\n",
                     safeX, safeZ, pass+1, rep+1);
            hal.stream.write(dbg);

            if(fabsf(target[X_AXIS] - safeX) > 0.0001f ||
               fabsf(target[Z_AXIS] - safeZ) > 0.0001f)
            {
                target[X_AXIS] = safeX;
                target[Z_AXIS] = safeZ;
                mc_line(target, &plan_g0);
            }

            // ------------------------------------------------------------
            // 2) DEPTH: avanzamento in X alla profondità corrente
            // ------------------------------------------------------------
            target[X_AXIS] = X_start + current_depth;
            target[Z_AXIS] = Z_start + R;

            snprintf(dbg, sizeof(dbg),
                     "M800 G1 DEPTH:  X=%.3f Z=%.3f\r\n",
                     target[X_AXIS], target[Z_AXIS]);
            hal.stream.write(dbg);

            mc_line(target, &plan_g1);

            // ------------------------------------------------------------
            // 3) LENGTH: avanzamento in Z lungo la cava
            // ------------------------------------------------------------
            target[Z_AXIS] = Z_start + L;

            snprintf(dbg, sizeof(dbg),
                     "M800 G1 LENGTH: X=%.3f Z=%.3f\r\n",
                     target[X_AXIS], target[Z_AXIS]);
            hal.stream.write(dbg);

            mc_line(target, &plan_g1);

            // ------------------------------------------------------------
            // 4) BACKX: ritorno rapido in X
            // ------------------------------------------------------------
            target[X_AXIS] = X_start;

            snprintf(dbg, sizeof(dbg),
                     "M800 G0 BACKX:  X=%.3f Z=%.3f\r\n",
                     target[X_AXIS], target[Z_AXIS]);
            hal.stream.write(dbg);

            mc_line(target, &plan_g0);

            // ------------------------------------------------------------
            // 5) BACKZ: ritorno rapido in Z
            // ------------------------------------------------------------
            float backZ = Z_start + R;

            if(fabsf(target[Z_AXIS] - backZ) > 0.0001f)
            {
                target[Z_AXIS] = backZ;

                snprintf(dbg, sizeof(dbg),
                         "M800 G0 BACKZ:  X=%.3f Z=%.3f\r\n",
                         target[X_AXIS], target[Z_AXIS]);
                hal.stream.write(dbg);

                mc_line(target, &plan_g0);
            }
        }
    }

    // -------------------------------------------------------------------------
    //  Passata finale
    // -------------------------------------------------------------------------
    target[X_AXIS] = X_start - D;
    target[Z_AXIS] = Z_start + R;

    snprintf(dbg, sizeof(dbg),
             "M800 FINAL:   X=%.3f Z=%.3f\r\n",
             target[X_AXIS], target[Z_AXIS]);
    hal.stream.write(dbg);

    mc_line(target, &plan_g1);

    // -------------------------------------------------------------------------
    //  Ritorno alla posizione iniziale (solo se H1)
    // -------------------------------------------------------------------------
    if(return_home) {

        target[X_AXIS] = X_start;
        target[Z_AXIS] = Z_start;

        snprintf(dbg, sizeof(dbg),
                 "M800 RETURN:  X=%.3f Z=%.3f\r\n",
                 target[X_AXIS], target[Z_AXIS]);
        hal.stream.write(dbg);

        mc_line(target, &plan_g0);
    }

    // -------------------------------------------------------------------------
    //  Sincronizza e riallinea gc_state
    // -------------------------------------------------------------------------
    protocol_buffer_synchronize();

    gc_state.position[X_AXIS] = sys.position[X_AXIS];
    gc_state.position[Z_AXIS] = sys.position[Z_AXIS];

    hal.stream.write("M800 CYCLE END\r\n");
}


// -----------------------------------------------------------------------------
//  INIT
// -----------------------------------------------------------------------------

void my_plugin_init(void)
{
    memcpy(&user_mcode_prev, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check    = m800_check;
    grbl.user_mcode.validate = m800_validate;
    grbl.user_mcode.execute  = m800_execute;
}

#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"
#include "fixextra.h"
#include "fixmatrix.h"
#include "fixquat.h"
#include "fixvector3d.h"

#include "controllers/control_lib.h"
#include "controllers/controller_att_nac.h"
#include "drivers/drv_status_io.h"
#include "params.h"
#include "mixer.h"
#include "safety.h"

static mf16 theta; // [Ixx; Iyy; Izz]

static void controller_att_nac_calc_Y(mf16* Y, const v3d* w, const v3d* wd, const v3d* dwd) {
	// H(q)ddq + C(q,dq)dq = Y(q,dq,ddq)a
	//Y = diag(dwd) + vee_up(w)*diag(wd);

	Y->rows = Y->columns = 3;
	Y->errors = 0;
	//Unroll the operation to save on multiplying lots of zeros
	Y->data[0][0] = dwd->x;
	Y->data[0][1] = fix16_mul(-w->z, wd->y);
	Y->data[0][2] = fix16_mul( w->y, wd->z);
	Y->data[1][0] = fix16_mul( w->z, wd->x);
	Y->data[1][1] = dwd->y;
	Y->data[1][2] = fix16_mul(-w->x, wd->z);
	Y->data[2][0] = fix16_mul(-w->y, wd->x);
	Y->data[2][1] = fix16_mul( w->x, wd->y);
	Y->data[2][2] = dwd->z;
}

//XXX: This is also our init()
void controller_att_nac_init( void ) {
	controller_att_nac_reset();
}

void controller_att_nac_reset( void ) {
	theta.rows = 3;
	theta.columns = 1;

	//Do a full clear just in case
	mf16_fill(&theta, 0);
	const fix16_t prescale = get_param_fix16( PARAM_MC_NAC_T0_PRESCALER );
	theta.data[0][0] = fix16_mul( prescale, get_param_fix16( PARAM_MC_NAC_T0_IXX ));
	theta.data[1][0] = fix16_mul( prescale, get_param_fix16( PARAM_MC_NAC_T0_IYY ));
	theta.data[2][0] = fix16_mul( prescale, get_param_fix16( PARAM_MC_NAC_T0_IZZ ));
}

void controller_att_nac_save_parameters( void ) {
	const fix16_t prescale = get_param_fix16( PARAM_MC_NAC_T0_PRESCALER );
	set_param_fix16( PARAM_MC_NAC_T0_IXX, fix16_div( theta.data[0][0], prescale));
	set_param_fix16( PARAM_MC_NAC_T0_IYY, fix16_div( theta.data[1][0], prescale));
	set_param_fix16( PARAM_MC_NAC_T0_IZZ, fix16_div( theta.data[2][0], prescale));
}

/*
static void controller_att_nac_tau_to_c(v3d* c, v3d* tau) {
	const fix16_t Tmax = get_param_fix16( PARAM_NAC_TMAX);
	const fix16_t Dmax = get_param_fix16( PARAM_NAC_TMAX);
	const fix16_t l = get_param_fix16( PARAM_NAC_ARM_LENGTH);
	const uint8_t n = mixer_get_num_motors();
}
*/

void controller_att_nac_step( v3d* c, v3d* rates_ref, const command_input_t* input, const state_t* state, const fix16_t dt ) {
    // Control (Adaptive Control)
	//==-- Control Parameters
	//PARAM_NAC_W0R: 20.0
	//PARAM_NAC_DZ_EW: deg2rad(0.1)
	//PARAM_NAC_DZ_ER: deg2rad(1.0)
	const fix16_t w0r = get_param_fix16( PARAM_MC_NAC_W0R );
	const fix16_t prescale = get_param_fix16( PARAM_MC_NAC_T0_PRESCALER );
	// Adaption gain (a -> gamma)
	// Set these based on the expected closed loop frequencies (p) of the
	// controller that interract with these parameters, as well as a
	// scaling term (p0) that is dependant on the accuracy of our initial
	// estimates (system will be unstable if this is too high).
	//p = [w0r;w0r;w0r;w0p];
	//a = p0*p/max(p);
	//Gamma = diag(a);
	mf16 Gamma = {.rows = 3, .columns = 3, .errors = 0};
	mf16_fill_diagonal(&Gamma, w0r);
	// Dead-zone
	// These should be set depending on state noise
	const fix16_t dz_detla_ew = get_param_fix16( PARAM_NAC_DZ_EW )
							  + fix16_mul(w0r,get_param_fix16( PARAM_NAC_DZ_ER ));

	//==-- Input Signals
	qf16 qe = QF16_NO_ROT; //start with no angle error (R==R_sp==Identity)
	v3d eRb = V3D_ZERO;
	v3d ew;
	v3d ewd;
	/*
	XXX: Notes for pure control modes:
	//For Stab (heading-hold):
		//eR = R'*R_sp;
		eRb = control_lib_q_att_error(...);
		v3d_sub(&ew, &v3d0, &w);		//ew = eR*w_sp - w; w_sp => 0
		ewd = v3d0;						//eR*wd_sp - vee_up(w)*eR*w_sp; wd_sp, w_sp => 0
	//For Acro
		eRb = v3d0;
		v3d_sub(&ew, &w_sp, &w);	//ew = eR*w_sp - w; eR => eye(3) for acro
		v3d_cross(&ewd, &w, &w_sp);	//eR*wd_sp - vee_up(w)*eR*w_sp; eR => eye(3) for acro => w x w_sp
	*/

	// If we should listen to attitude input
	if ( !( input->input_mask & CMD_IN_IGNORE_ATTITUDE ) ) {
		fix16_t yaw_w = get_param_fix16( PARAM_MC_ANGLE_YAW_W );

		// If we are going to override the calculated yaw rate, just ignore it
		if ( !( input->input_mask & CMD_IN_IGNORE_YAW_RATE ) ) {
			yaw_w = 0;
		}

		control_lib_q_att_error( &eRb,
								 &qe,
								 &(input->q),
								 &(state->q),
								 yaw_w );
	}

	// Rates
	v3d w_sp;

	// Roll
	if ( !( input->input_mask & CMD_IN_IGNORE_ROLL_RATE ) ) {
		// Use the commanded roll rate goal
		w_sp.x = input->r;
	}

	// Pitch
	if ( !( input->input_mask & CMD_IN_IGNORE_PITCH_RATE ) ) {
		// Use the commanded pitch rate goal
		w_sp.y = input->p;
	}

	// Yaw
	if ( !( input->input_mask & CMD_IN_IGNORE_YAW_RATE ) ) {
		// Use the commanded yaw rate goal
		w_sp.z = input->y;
	} else {
		// XXX: Better yaw tracking in stab mode
		// If we're in offboard mode, and we aren't going to override yaw rate, and
		// we want to fuse
		// XXX: Ideally this would be handled as an additional case using the IGNORE
		// flags, somehow...
		if ( ( _system_status.control_mode == MAIN_MODE_OFFBOARD ) &&
			 get_param_uint( PARAM_CONTROL_OB_FUSE_YAW_RATE ) ) {
			// Add in the additional yaw rate input
			// TODO: This is more of a hack. Needs to be conerted from euler rate to body rates (should effect goal_y, goal_p, and goal_r)
			w_sp.z = fix16_add( rates_ref->z, input->y );
		}
	}

	//Save our rates inputs
	*rates_ref = w_sp;

	// Calculate our other error terms (can't do shortcuts without working out exactly the mode we're in)
	//ew = eR*w_sp - w;
	v3d eRw_sp;
	qf16_rotate(&eRw_sp, &qe, &w_sp);
	v3d_sub(&ew, &eRw_sp, &(state->w));

	//ewd = eR*wd_sp - vee_up(w)*eR*w_sp; wd_sp == [0;0;0] => ewd = -vee_up(w)*eR*w_sp = -cross(w,eR*w_sp)
    v3d_cross(&ewd, &(state->w), &eRw_sp);
	v3d_inv(&ewd, &ewd);


	//==-- Variable Preparation
    // A = [w0r,   0,   0,
	//		 0, w0r,   0,
	//		 0,   0, w0r];
	//s = [-ew] + A*[-c(CONTROL_R_E,i)];
	//dqr = [-eR_s*c(CONTROL_WXYZ_B,i)] - A*[-c(CONTROL_R_E,i)];
	//ddqr = [ewd] - A*[-ew];
	v3d n_AeRb;
	v3d_mul_s(&n_AeRb, &eRb, -w0r);
	v3d n_ew;
	v3d_inv(&n_ew, &ew);
	v3d s;
	v3d_add(&s, &n_ew, &n_AeRb);

	v3d dqr;
	v3d_inv(&dqr, &n_AeRb);	//CONTROL_WXYZ_B => [0,0,0]

	v3d An_ew;
	v3d_mul_s(&An_ew, &n_ew, w0r);

	v3d ddqr;
	v3d_sub(&ddqr, &ewd, &An_ew);
	//v = ddqr - A*s + [0;0;0;-thrust_scale*R_sp_c'*g_vec];
	v3d As;
	v3d_mul_s(&As, &s, w0r);
	v3d v;
	v3d_sub(&v, &ddqr, &As);

	//==-- Calculate Regressor
	mf16 Ym;
	controller_att_nac_calc_Y(&Ym, &(state->w), &dqr, &v);

	//==-- Control law
	mf16 mtau; //u
	mf16_mul(&mtau, &Ym, &theta);

	//thetad = -Gamma*Ym'*s
	mf16 GYm;
	mf16_mul_bt(&GYm, &Gamma, &Ym);
	mf16 nms = {.rows = 3, .columns = 1, .errors = 0};
	nms.data[0][0] = -s.x;
	nms.data[1][0] = -s.y;
	nms.data[2][0] = -s.z;
	mf16 thetad;
	mf16_mul(&thetad, &GYm, &nms);

    //==-- Parameter update refinement
	//Check if we're using a deadzone
    if( dz_detla_ew > 0 ) {
		if( fix16_abs( v3d_norm( &s ) ) < dz_detla_ew ) {
			mf16_fill( &thetad, 0 );	//In dead-zone, don't update parameters
		}
	}

	// Propogate parameters
	mf16_mul_s(&thetad, &thetad, dt);
	mf16_add(&theta, &theta, &thetad);

	//==-- Fill in control vector
	if( (mtau.rows == 3) && ( mtau.columns == 1) ) {
		v3d tau;
		tau.x = fix16_div(mtau.data[0][0],prescale);
		tau.y = fix16_div(mtau.data[1][0],prescale);
		tau.z = fix16_div(mtau.data[2][0],prescale);

		//XXX: 	Could actually map the parametes properly
		//		with thrust values, etc, but there is no
		//		point if we're not relying on "real" params
		//		to be found. This is really only needed if
		//		the full 3D pose controller is to be used.
		//		So, save some time computing and accept that
		//		Ixx, etc., will only be related to the real
		//		values, and not the actual values (related
		//		by some function of T, la, etc.).
		//controller_att_nac_tau_to_c(c, &tau);
		*c = tau;
	} else {
		safety_request_state(MAV_STATE_EMERGENCY);

		*c = V3D_ZERO;

		mavlink_queue_broadcast_error( "NAC: Control math failed" );
		status_buzzer_failure();
	}
}

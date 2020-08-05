#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"
#include "fixextra.h"

static mf16 theta; // [Ixx; Iyy; Izz]

static void nac_calc_Y(mf16* Y, const v3d* w, const v3d* wd, const v3d* dwd) {
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
	Y->data[2][1] = fix16_mul( w->x, wd->y)
	Y->data[2][2] = dwd->z;
}

//XXX: This is also our init()
void nac_reset() {
	theta.rows = 3;
	theta.cols = 1;

	//Do a full clear just in case
	for(int i=0; i < theta.rows; i++)
		for(int j=0; j < theta.cols; j++)
			theta.data[i][j] = 0;

	//PARAM_NAC_T0_IXX: 0.02961
	//PARAM_NAC_T0_IYY: 0.02961
	//PARAM_NAC_T0_IZZ: 0.05342
	theta.data[0][0] = get_param_fix16( PARAM_NAC_T0_IXX );
	theta.data[1][0] = get_param_fix16( PARAM_NAC_T0_IYY );
	theta.data[2][0] = get_param_fix16( PARAM_NAC_T0_IZZ );
}

void nac_step(v3d *tau, const fix16_t dt) {
    // Control (Adaptive Control)
	//==-- Control Parameters
	//PARAM_NAC_W0R: 20.0
	//PARAM_NAC_DZ_EW: deg2rad(0.1)
	//PARAM_NAC_DZ_ER: deg2rad(1.0)
	const v3d v3d0 = {0,0,0};
	const fix16_t w0r = get_param_fix16( PARAM_NAC_W0R );
	// Adaption gain (a -> gamma)
	// Set these based on the expected closed loop frequencies (p) of the
	// controller that interract with these parameters, as well as a
	// scaling term (p0) that is dependant on the accuracy of our initial
	// estimates (system will be unstable if this is too high).
	//p = [w0r;w0r;w0r;w0p];
	//a = p0*p/max(p);
	//Gamma = diag(a);
	mf16 Gamma;
	Gamma.rows = Gamma.cols = 3;	//Filled by parameters on control loop
	mf16_fill_diagonal(&Gamma, w0r);
	// Dead-zone
	// These should be set depending on state noise
	const fix16_t dz_detla_ew = get_param_fix16( PARAM_NAC_DZ_EW )
							  + fix16_mul(w0r,get_param_fix16( PARAM_NAC_DZ_ER ));

	//==-- Input Signals
	v3d eRb;
	v3d ew;
	v3d ewd;
	//For Stab:
	//eR = R'*R_sp;
	eRb = q_att_error(...)
	v3d_sub(&ew, &v3d0, &w);		//ew = eR*w_sp - w; w_sp => 0
    ewd = v3d0;						//eR*wd_sp - vee_up(w)*eR*w_sp; wd_sp, w_sp => 0
	//For Acro
	eRb = v3d0;
	v3d_sub(&ew, &w_sp, &w);	//ew = eR*w_sp - w; eR => eye(3) for acro
    v3d_cross(&ewd, &w, &w_sp);	//eR*wd_sp - vee_up(w)*eR*w_sp; eR => eye(3) for acro => w x w_sp

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
	control_adaptive_calc_Y(&Ym, &w, &dqr, &v);

	//==-- Control law
	mf16 mtau; //u
	mf16_mul(&mtau, &Ym, &theta);

	//thetad = -Gamma*Ym'*s
	mf16 GYm;
	mf16_mul_bt(&GYm, &Gamma, &Ym);
	mf16 nms;
	nms.rows = 3;
	nms.cols = 1;
	nms.data[0][0] = -s.x;
	nms.data[1][0] = -s.y;
	nms.data[2][0] = -s.z;
	mf16 thetad;
	mf16_mul(&thetad, &GYm, &nms);

    //==-- Parameter update refinement
	//Check if we're using a deadzone
    if(dz_detla_ew > 0) {
		if(fix16_abs(v3d_norm(s)) < dz_detla_ew) {
			mf16_fill(&thetad, 0);	//In dead-zone, don't update parameters
		}
	}

	// Propogate parameters
	mf16_mul_s(&thetad, &thetad, dt);
	mf16_add(&theta, &theta, &thetad);

	//==-- Fill in control vector
	if(mtau.rows == 3) &&( mtau.cols == 1) {
		tau->x = mtau[0][0];
		tau->y = mtau[1][0];
		tau->z = mtau[2][0];
	} else {
		//TODO: Throw error!
	}
}

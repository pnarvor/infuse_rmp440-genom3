/*
 * Copyright (c) 1991-2009 CNRS/LAAS
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "rmp440_c_types.h"
#include "orMathLib.h"

/*
 * Odometry error evaluation
 */


#define Coef 0.002			  /* pourcentage d'erreur sur le
					   * deplacement de chacune des
					   * roues; la valeur indiquee est
					   * plus qu'aproximative !! A mon
					   * avis il faut faire des essais
					   * ... (c'est beau la science !!!) */

/* valeur du test du Chi2 a 1 degres de liberte a 99% */
#define GAUSS_99 3.0 /* A peu près ! */
#define SIGMA3_TO_VARIANCE(x) (SQR((x)/GAUSS_99))
#define VARIANCE_TO_SIGMA3(x)   (GAUSS_99*sqrt(x))

static int dms_plus(const double *m1, const double *m2, double *m_res,
		int l, int c);
static int dmfsf_jmult2(const double *m1, const double *m2, double *m_res,
		int l1, int c1, int l2, int c2);

static void calc_nouv_pos(or_genpos_cart_state *robot, double dS, double dTh,  double LaX,
			  double *jxr, double *jdep);
static void Calc_var_whee(double dS, double dTh, double coeff, double LaX,
			  double *vdep);

/**
 ** fonction de calcul de l'odometrie
 **
 ** entree: posId - ancienne position + incertitude
 **                 deplacements mesures
 **
 ** sortie: posId - nouvelle position + incertitude
 **/

void 
odoProba(or_genpos_cart_state *robot, 
    or_genpos_cart_config_var *var,  	
    double lax, 	/* entre-axe */
    double coeff,
    double period)
{
	/* jacobiennes (J_h_Xr,J_h_dR) */
	static double J_h_Xr[9], J_h_dR[6];
	
	/* tableaux auxilliaires*/
	static double aux[9];
	static double V_dep[3];
	
	double dS, dTh;
	
	dS = robot->v * period;
	dTh = robot->w * period;
	
	/*
	 * calcul des jacobiennes en fonction des deplacements des roues.
	 */
	calc_nouv_pos(robot, dS, dTh, lax, J_h_Xr, J_h_dR);
	
	/*
	 * variance sur le deplacement
	 */
	Calc_var_whee(dS, dTh, coeff, lax, V_dep);
	
	/*
	 * calcul de la variance sur Xrob
	 */
	dmfsf_jmult2(J_h_dR, V_dep, aux, 3, 2, 2, 2);
	dmfsf_jmult2(J_h_Xr, var->var, var->var, 3, 3, 3, 3);
	dms_plus(aux, var->var, var->var, 3, 3);
#if 0
	printf ("dS %f dT %f coeff %f errx %f erry %f errt %f\n",
	    dS, dTh, coeff, VARIANCE_TO_SIGMA3(var->var[0]), 
	    VARIANCE_TO_SIGMA3(var->var[2]), VARIANCE_TO_SIGMA3(var->var[5]));
#endif
}


static void 
calc_nouv_pos(or_genpos_cart_state *robot, double dS, double dTh,  double LaX,
    double *jxr, double *jdep)
{ 
	double c, s, thetaMoy;

	thetaMoy = robot->theta + dTh/2.;
	c = cos(thetaMoy);
	s = sin(thetaMoy);
	
	robot->theta = angleLimit(robot->theta + dTh);
	robot->xRob += dS *c;
	robot->yRob += dS *s;

	/* jacobiennes */
	
	/* premiere ligne jxr */
	jxr[0] = 1.0;
	jxr[1] = 0;
	jxr[2] = -dS * s;
	/* deuxieme ligne jxr */
	jxr[3] = 0;
	jxr[4] = 1.0;
	jxr[5] = dS * c;
	/* troisieme ligne jxr */
	jxr[6] = 0;
	jxr[7] = 0;
	jxr[8] = 1.0;
	
	/* premiere ligne de jdep */
	jdep[0] = c/2. + dS*s/LaX;
	jdep[1] = c/2. - dS*s/LaX;
	/* deuxieme ligne de jdep */
	jdep[2] = s/2. - dS*c/LaX;
	jdep[3] = s/2. + dS*c/LaX;
	/* troisieme ligne de jdep */
	jdep[4] = -1/LaX;
	jdep[5] = 1/LaX;
}

static void 
Calc_var_whee(double dS, double dTh,
    double coeff, double LaX,
    double *vdep)
{
	double dsR, dsL;
	
	dsR = dS + LaX/2.0 * dTh;
	dsL = dS - LaX/2.0 * dTh;
	
	vdep[0] = coeff * dsL * dsL;
	vdep[1] = 0;
	vdep[2] = coeff * dsR * dsR;
}

#define MATELT(mat,i,j,c)   ( *((mat) + (c) * (i) + (j)) )
#define INDEX_SMATELT(i,j)					\
((i)<(j) ? ((j)*((j)+1))/2 + (i) : ((i)*((i)+1))/2 +(j))
#define SMATELT(mat,i,j,n) *((mat) + INDEX_SMATELT((i),(j)))
#define MATSIZE(l,c)        ((l) * (c))
#define SMATSIZE(n,c)       (((n) * ((n) + 1)) / 2)

static int
dms_plus(const double *m1, const double *m2, double *m_res, int l, int c)
{
	int k;

	if (l != c)
	{
		fprintf (stderr, "dms : matrice symetrique non carree !!!\n");
		return -1;
	}

	for (k = 0; k < SMATSIZE(l, c); k++)
		*m_res++ = *m1++ + *m2++;
	return 0;
}

static int
dmfsf_jmult2 (const double *m1, const double *m2, double *m_res,
		int l1, int c1, int l2, int c2)
{

	double *temp, *res;
	double *aux = NULL;
	double *aux2 = NULL;
	int i,j,k,flag = 0;
	double val;

	if (l2 != c2 || c1 != c2)
	{
		fprintf(stderr, "%s : tailles incorrectes \n", __func__);
		return -1;
	}

	if (m_res == m1 || m_res == m2)
	{
		temp = malloc((l1*c2 + l1*(l1+1)/2) * sizeof(double));
		if (temp == NULL) {
			fprintf(stderr, "%s : Can't allocate memory for temp\n", __func__);
			return -1;
		}
		aux = temp;
		res = temp + l1*c2;
		aux2 = res;
		flag = 1;
	}
	else
	{
		temp = malloc((l1 * c2) * sizeof(double));
		if (temp == NULL) {
			fprintf(stderr, "%s : Can't allocate memory for temp\n", __func__);
			return -1;
		}
		aux = temp;
		res = m_res;
	}


	for (i = 0; i < l1; i++)
		for (j = 0; j < c2; j++)
		{
			val = 0.;
			for (k = 0; k < c1; k++)
				val += MATELT (m1, i, k, c1) * SMATELT (m2, k, j, c2);
			*temp++ = val;
		}
	temp = aux;

	for (j = 0; j < l1; j++)
		for (i = 0; i <= j; i++)
		{
			val = 0.;
			for (k = 0; k < c2; k++)
				val += MATELT (temp, i, k, c2) * MATELT (m1, j, k, c1);
			*res++ = val;
		}

	if (flag)
	{
		res = aux2;
		for (i = 0; i < SMATSIZE (l1, l1); i++)
			*m_res++ = *res++;
	}

	free(temp);

	return 0;
}

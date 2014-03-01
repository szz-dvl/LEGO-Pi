/***********************************************/
/* T.Kouya's GSL sample program collection     */
/*            Interpolation of Discrete Points */
/*                   Written by Tomonori Kouya */
/*                                             */
/* Version 0.01: 2007-10-04                    */
/***********************************************/
#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_bspline.h>
#include <stdbool.h>
#include <string.h>

			       /* Santi Stuff */
#define L 10
#define Q 15
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

double vel    [L] = {10, 30, 50, 70, 90, 110, 130, 150, 170, 200};
double micras [L] = {6283.822603, 4201.347957, 3202.906810, 2581.099426, 2133.879297, 1816.458389, 1580.550189, 1401.993175, 1253.643091, 1046.488593};
double desv   [L] = {460.687865, 419.313832, 389.258212, 323.866784, 275.788692, 236.213653, 216.627822, 212.979838, 208.559258, 222.149722};
double guess        [L] = {20, 40, 60, 80, 100, 120, 140, 160, 180, 195};

double vel15    [Q] = {10, 23, 36, 49, 62, 75, 88, 101, 114, 127, 140, 153, 166, 179, 200};
double micras15 [Q] = {6286.651533, 4733.501628, 3862.326328, 3282.145680, 2849.344128, 2478.517935, 2162.523440, 1936.191410, 1764.641264, 1638.004420, 1519.949160, 1410.563553, 1298.026937, 1208.759614, 1050.160080};
double desv15   [Q] = {456.095402, 440.911585, 432.034228, 407.277184, 341.646183, 313.247809, 300.153527, 268.296274, 239.802972, 203.658592, 190.593414, 194.946423, 205.931028, 209.402317, 223.484705};
double guess15        [Q] = {16, 29, 42, 55, 68, 81, 94, 107, 120, 133, 146, 159, 172, 185, 198};

double vel20    [L*2] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200};
double micras20 [L*2] = {6291.670787, 4974.964956, 4246.034515, 3683.674392, 3231.695571, 2894.098608, 2589.313464, 2329.800382, 2130.462146, 1960.860620, 1801.624728, 1698.443527, 1593.361824, 1493.730624, 1416.815650, 1352.296069, 1263.664475, 1179.860561, 1127.353635, 1061.691239};
double desv20   [L*2] = {447.969237, 446.202126, 434.001658, 412.067526, 387.965182, 355.353557, 333.029812, 314.236926, 276.861747, 266.043386, 261.272787, 225.281757, 206.351683, 206.628301, 198.755767, 198.788691, 206.969957, 215.030614, 217.598542, 219.911220};
double guess20        [L*2] = {15, 25, 35, 45, 55, 65, 75, 85, 95, 105, 115, 125, 135, 145, 155, 165, 175, 185, 195, 199};

double vel20_cp    [L*2] = {0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00, 1.10, 1.20, 1.30, 1.40, 1.50, 1.60, 1.70, 1.80, 1.90, 2.00};
double micras20_cp [L*2] = {62.91670787, 49.74964956, 42.46034515, 36.83674392, 32.31695571, 28.94098608, 25.89313464, 23.29800382, 21.30462146, 19.60860620, 18.01624728, 16.98443527, 15.93361824, 14.93730624, 14.16815650, 13.52296069, 12.63664475, 11.79860561, 11.27353635, 10.61691239};
double desv20_cp   [L*2] = {4.47969237, 4.46202126, 4.34001658, 4.12067526, 3.87965182, 3.55353557, 3.33029812, 3.14236926, 2.76861747, 2.66043386, 2.61272787, 2.25281757, 2.06351683, 2.06628301, 1.98755767, 1.98788691, 2.06969957, 2.15030614, 2.17598542, 2.19911220};
double guess20_cp        [L*2] = {0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.05, 1.15, 1.25, 1.35, 1.45, 1.55, 1.65, 1.75, 1.85, 1.95, 1.99};

/* END Santi Stuff */

void interpol ( double puntsX[], double puntsY[], double puntsINT [], double res[], size_t dimXY, int dimRES, int type, char *used ) {

  /*
     puntsX   > puntos abscisas, el vector debe ser de tamaño "dimXY".
     puntsY   > puntos ordenadas, el vector debe ser de tamaño "dimXY".
     puntsINT > puntos que queremos allar por interpolación, de tamaño "dimRES".
     res      > resultado de la interpolación para los puntos especificados en "puntsINT", lógicamente de tamaño "dimRES".
     type     > tipo de interpolación usada: (http://www.gnu.org/software/gsl/manual/html_node/Interpolation-Types.html#Interpolation-Types)

                0 > linear interpolation. (puede que para micras sirva...). >> No aproxima mal, pero nadie puede asegurar, sobretodo para desviaciones que tratemos con rectas.
		1 > polinomycal interpolation. >> Runge Phenomenon...  ¡¡¡¡no quieees cerveza....anda y vete por ahíííí...!!!!
		2 > cspline (Natural) interpolation. >> idem AKIMA pero un pelo peor.
		3 > cspline (Periodical) interpolation. >> idem AKIMA pero un pelo peor. 
		4 > akima (Natural) interpolation. >> Buena pero en los extremos se va de bareta un pelín.
		5 > akima (Periodical) interpolation. >>> ESTA PARECE SER LA MEJOR, en los extremos aproxima incluso mejor que spline periódica.

     dimXY    > Se define an las explicaciones anteriores.
     dimRES   > Se define an las explicaciones anteriores.

   */

  gsl_interp *workspace;
  gsl_interp_accel *accel;
  int i;
  bool init = true;

  switch(type) {
  case 0:
    {
      workspace = gsl_interp_alloc(gsl_interp_linear, dimXY);
      strcpy(used, "LINEAR INTERPOLATION");
    }
    break;;
  case 1:
    {
      workspace = gsl_interp_alloc(gsl_interp_polynomial, dimXY);
      strcpy(used, "POLYNOMICAL INTERPOLATION");
    }
    break;;
  case 2:
    {
      workspace = gsl_interp_alloc(gsl_interp_cspline, dimXY);
      strcpy(used,"NATURAL SPLINE INTERPOLATION");
    }
    break;;
  case 3:
    {
      workspace = gsl_interp_alloc(gsl_interp_cspline_periodic, dimXY);
      strcpy(used, "PERIODICAL SPLINE INTERPOLATION");
    }
    break;;
  case 4:
    {
      workspace = gsl_interp_alloc(gsl_interp_akima, dimXY);
      strcpy(used, "NATURAL AKIMA INTERPOLATION");
    }
    break;;
  case 5:
    {
      workspace = gsl_interp_alloc(gsl_interp_akima_periodic, dimXY);
      strcpy(used, "PERIODICAL AKIMA INTERPOLATION");
    }
    break;;

  default:
    init = false;
    break;;
  }
      
  if(init) {
    gsl_interp_init(workspace, puntsX, puntsY, dimXY);    
    accel = gsl_interp_accel_alloc();

    for(i = 0; i < dimRES; i++)
      res[i] = gsl_interp_eval(workspace, puntsX, puntsY, puntsINT[i], accel); //¿accel? --> si ho he entés bé per els ultims punts ens servirà.
    
    gsl_interp_accel_free(accel);
    gsl_interp_free(workspace);  
  }
}

int main(int argc, char * argv[]) {

  int tam, i;
  double *vels, *micrs, *desvs, *ask;
  char *used = malloc(35*sizeof(char));
  int tam_ref = argc < 2 ? 1 : atoi(argv[1]);//default 10 samples
  int type = argc < 3 ? 1 : atoi(argv[2]); //default polynomical.
  bool check = argc < 4 ? false : atoi(argv[3]) != 0 ? true : false; //default NO CHECK, SI GUESS.

  switch(tam_ref) {
  case 1:
    {
      tam = L;
      vels = vel;
      micrs = micras;
      desvs = desv;
      ask = guess;
    }
    break;;
  case 2:
    {
      tam = Q;
      vels = vel15;
      micrs = micras15;
      desvs = desv15;
      ask = guess15;
    }
    break;;
  case 3 :
    {
      tam = L*2;
      vels = vel20;
      micrs = micras20;
      desvs = desv20;
      ask = guess20;
    }
    break;;
  case 4:
    {
      tam = L*2;
      vels = vel20_cp;
      micrs = micras20_cp;
      desvs = desv20_cp;
      ask = guess20_cp;
    }
    break;;
  default:
    exit(0);
    break;;
  }
		     
  double res_m[tam];
  double res_d[tam];

  if(!check) {		     

    interpol (vels,micrs,ask,res_m,tam,tam,type,used);
    interpol (vels,desvs,ask,res_d,tam,tam,type,used);

    printf("\nPrinting results for %d samples%s, using: %s\n", tam, tam_ref == 4 ? " CAPPED" : "", used);
    for (i = 0; i < tam; i++){
      if (i != tam-1)
	printf("VEL = %3.0f, MICRAS = %f, DESV = %f\n%sVEL = %3.0f, MICRAS = %f, DESV = %f%s\n", vels[i], micrs[i], desvs[i], KBLU, ask[i], res_m[i], res_d[i], KNRM);
      else
	printf("%sVEL = %3.0f, MICRAS = %f, DESV = %f%s\nVEL = %3.0f, MICRAS = %f, DESV = %f\n", KBLU, ask[i], res_m[i], res_d[i], KNRM, vels[i], micrs[i], desvs[i]);
    }

  } else {

    interpol (vels,micrs,vels,res_m,tam,tam,type,used);
    interpol (vels,desvs,vels,res_d,tam,tam,type,used);

    printf("\nPrinting results for %d samples%s, using: %s\n", tam, tam_ref == 4 ? " CAPPED" : "", used);
    for (i = 0; i < tam; i++){
      if (i != tam-1)
	printf("VEL = %3.0f, MICRAS = %f, DESV = %f\n%sVEL = %3.0f, MICRAS = %f, DESV = %f%s\n", vels[i], micrs[i], desvs[i], KBLU, vels[i], res_m[i], res_d[i], KNRM);
      else
	printf("%sVEL = %3.0f, MICRAS = %f, DESV = %f%s\nVEL = %3.0f, MICRAS = %f, DESV = %f\n", KBLU, vels[i], res_m[i], res_d[i], KNRM, vels[i], micrs[i], desvs[i]);
    }
  }
  return 0;
}


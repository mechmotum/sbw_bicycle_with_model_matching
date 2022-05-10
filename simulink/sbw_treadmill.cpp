/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    DifferentialState x_P;
    DifferentialState y_P;
    DifferentialState psi;
    DifferentialState phi;
    DifferentialState delta;
    DifferentialState d_phi;
    DifferentialState d_delta;
    Control T_delta;
    Disturbance W;
    BMatrix acadodata_M1;
    acadodata_M1.read( "sbw_treadmill_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "sbw_treadmill_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << x_P;
    acadodata_f2 << y_P;
    acadodata_f2 << psi;
    acadodata_f2 << phi;
    acadodata_f2 << delta;
    acadodata_f2 << d_phi;
    acadodata_f2 << d_delta;
    acadodata_f2 << T_delta;
    Function acadodata_f3;
    acadodata_f3 << x_P;
    acadodata_f3 << y_P;
    acadodata_f3 << psi;
    acadodata_f3 << phi;
    acadodata_f3 << delta;
    acadodata_f3 << d_phi;
    acadodata_f3 << d_delta;
    OCP ocp1(0, 2, 160);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo((-3.49065850398865895610e-01) <= phi <= 3.49065850398865895610e-01);
    ocp1.subjectTo((-6.98131700797731791219e-01) <= delta <= 6.98131700797731791219e-01);
    ocp1.subjectTo((-7.00000000000000000000e+00) <= T_delta <= 7.00000000000000000000e+00);
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(x_P) == 5.50000000000000000000e+00*cos(psi);
    acadodata_f1 << dot(y_P) == 5.50000000000000000000e+00*sin(psi);
    acadodata_f1 << dot(psi) == (5.50000000000000000000e+00*delta+6.90000000000000057732e-02*d_delta)/1.12000000000000010658e+00*9.21060994002885102816e-01;
    acadodata_f1 << dot(phi) == d_phi;
    acadodata_f1 << dot(delta) == d_delta;
    acadodata_f1 << dot(d_phi) == ((-1.05729096715719061983e+00)*9.81000000000000049738e+00*delta-(-2.18955339435227443801e+02)*phi+(-2.66346580551956613192e+01)*phi-(-4.38456797022881317183e-01)*9.81000000000000049738e+00*delta+(-6.94584047861678710589e-01)*5.50000000000000000000e+00*d_phi-1.54700650219234758787e+00*T_delta+1.61666722759546455102e+00*5.50000000000000000000e+00*d_delta-1.85362781790660484660e+01*3.02500000000000000000e+01*delta+2.44738176966301512749e+00*3.02500000000000000000e+01*delta-5.50000000000000000000e+00*6.60545241203853095158e+00*d_delta)/2.32931485945728944387e+01;
    acadodata_f1 << dot(d_delta) == ((-1.35583503367983962562e+03)*phi-(-1.77017834263987515442e+03)*phi+(-2.71505178952045467256e+00)*9.81000000000000049738e+00*delta-(-4.61630720439466841754e+01)*5.50000000000000000000e+00*d_phi-(-7.02691045648822125713e+01)*9.81000000000000049738e+00*delta+1.02816315510000009681e+02*T_delta-1.07446069238605758756e+02*5.50000000000000000000e+00*d_delta+1.14782016341956406791e+02*3.02500000000000000000e+01*delta-1.62656573095520286643e+02*3.02500000000000000000e+01*delta+4.09028791745735134100e+01*5.50000000000000000000e+00*d_delta)/2.32931485945728944387e+01;

    ocp1.setModel( acadodata_f1 );


    ocp1.setNU( 1 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 0 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 320 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES3 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( HOTSTART_QP, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-04 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    options_flag = ExportModule1.set( GENERATE_SIMULINK_INTERFACE, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_SIMULINK_INTERFACE");
    options_flag = ExportModule1.set( GENERATE_MATLAB_INTERFACE, NO );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_MATLAB_INTERFACE");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "sbw_export" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 


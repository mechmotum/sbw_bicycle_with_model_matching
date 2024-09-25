'''
___[ kane_with_friction.py ]___
This script creates an parametric state space model 
of the Carvallo-Whipple bicycle model. Here this 
bicycle model has friction in the steering column.
This script is an adaptation of one of the test cases 
of the sympy library. See:
----
https://github.com/sympy/sympy/tree/master/sympy/physics/mechanics/tests -> kane3
https://docs.sympy.org/latest/modules/physics/mechanics/examples/bicycle_example.html
----
The only thing altered from that script is the 
inclusion of friction in the steer.
'''

import dill
from meijaard2kane3_conversion import meijaard2kane
from sympy import pprint, lambdify
from sympy.core.numbers import pi
from sympy.core.symbol import symbols
from sympy.functions.elementary.miscellaneous import sqrt
from sympy.functions.elementary.trigonometric import acos, sin, cos
from sympy.functions.elementary.hyperbolic import tanh
from sympy.matrices.dense import Matrix
from sympy.physics.mechanics import (ReferenceFrame, dynamicsymbols,
                                     KanesMethod, inertia, Point, RigidBody,
                                     dot, find_dynamicsymbols)
from numpy import linspace

def create_ss_cw_with_friction(friction_torque_fun):
    # Code to get equations of motion for a bicycle modeled as in:
    # J.P Meijaard, Jim M Papadopoulos, Andy Ruina and A.L Schwab. Linearized
    # dynamics equations for the balance and steer of a bicycle: a benchmark
    # and review. Proceedings of The Royal Society (2007) 463, 1955-1982
    # doi: 10.1098/rspa.2007.1857

    # Note that this code has been crudely ported from Autolev, which is the
    # reason for some of the unusual naming conventions. It was purposefully as
    # similar as possible in order to aide debugging.

    # Declare Coordinates & Speeds
    # Simple definitions for qdots - qd = u
    # Speeds are:
    # - u1: yaw frame ang. rate
    # - u2: roll frame ang. rate
    # - u3: rear wheel frame ang. rate (spinning motion)
    # - u4: frame ang. rate (pitching motion)
    # - u5: steering frame ang. rate
    # - u6: front wheel ang. rate (spinning motion)
    # Wheel positions are ignorable coordinates, so they are not introduced.
    q1, q2, q4, q5 = dynamicsymbols('q1 q2 q4 q5')
    q1d, q2d, q4d, q5d = dynamicsymbols('q1 q2 q4 q5', 1)
    u1, u2, u3, u4, u5, u6 = dynamicsymbols('u1 u2 u3 u4 u5 u6')
    u1d, u2d, u3d, u4d, u5d, u6d = dynamicsymbols('u1 u2 u3 u4 u5 u6', 1)

    # Declare System's Parameters
    WFrad, WRrad, htangle, forkoffset = symbols('WFrad WRrad htangle forkoffset')
    forklength, framelength, forkcg1 = symbols('forklength framelength forkcg1')
    forkcg3, framecg1, framecg3, Iwr11 = symbols('forkcg3 framecg1 framecg3 Iwr11')
    Iwr22, Iwf11, Iwf22, Iframe11 = symbols('Iwr22 Iwf11 Iwf22 Iframe11')
    Iframe22, Iframe33, Iframe31, Ifork11 = symbols('Iframe22 Iframe33 Iframe31 Ifork11')
    Ifork22, Ifork33, Ifork31, g = symbols('Ifork22 Ifork33 Ifork31 g')
    mframe, mfork, mwf, mwr = symbols('mframe mfork mwf mwr')

    # Set up reference frames for the system
    # N - inertial
    # Y - yaw
    # R - roll
    # WR - rear wheel, rotation angle is ignorable coordinate so not oriented
    # Frame - bicycle frame
    # TempFrame - statically rotated frame for easier reference inertia definition
    # Fork - bicycle fork
    # TempFork - statically rotated frame for easier reference inertia definition
    # WF - front wheel, again posses a ignorable coordinate
    N = ReferenceFrame('N')
    Y = N.orientnew('Y', 'Axis', [q1, N.z])
    R = Y.orientnew('R', 'Axis', [q2, Y.x])
    Frame = R.orientnew('Frame', 'Axis', [q4 + htangle, R.y])
    WR = ReferenceFrame('WR')
    TempFrame = Frame.orientnew('TempFrame', 'Axis', [-htangle, Frame.y])
    Fork = Frame.orientnew('Fork', 'Axis', [q5, Frame.x])
    TempFork = Fork.orientnew('TempFork', 'Axis', [-htangle, Fork.y])
    WF = ReferenceFrame('WF')

    # Kinematics of the Bicycle First block of code is forming the positions of
    # the relevant points
    # rear wheel contact -> rear wheel mass center -> frame mass center +
    # frame/fork connection -> fork mass center + front wheel mass center ->
    # front wheel contact point
    WR_cont = Point('WR_cont')
    WR_mc = WR_cont.locatenew('WR_mc', WRrad * R.z)
    Steer = WR_mc.locatenew('Steer', framelength * Frame.z)
    Frame_mc = WR_mc.locatenew('Frame_mc', - framecg1 * Frame.x
                                            + framecg3 * Frame.z)
    Fork_mc = Steer.locatenew('Fork_mc', - forkcg1 * Fork.x
                                            + forkcg3 * Fork.z)
    WF_mc = Steer.locatenew('WF_mc', forklength * Fork.x + forkoffset * Fork.z)
    WF_cont = WF_mc.locatenew('WF_cont', WFrad * (dot(Fork.y, Y.z) * Fork.y -
                                                    Y.z).normalize())

    # Set the angular velocity of each frame.
    # Angular accelerations end up being calculated automatically by
    # differentiating the angular velocities when first needed.
    # u1 is yaw rate
    # u2 is roll rate
    # u3 is rear wheel rate
    # u4 is frame pitch rate
    # u5 is fork steer rate
    # u6 is front wheel rate
    Y.set_ang_vel(N, u1 * Y.z)
    R.set_ang_vel(Y, u2 * R.x)
    WR.set_ang_vel(Frame, u3 * Frame.y)
    Frame.set_ang_vel(R, u4 * Frame.y)
    Fork.set_ang_vel(Frame, u5 * Fork.x)
    WF.set_ang_vel(Fork, u6 * Fork.y)

    # Form the velocities of the previously defined points, using the 2 - point
    # theorem (written out by hand here).  Accelerations again are calculated
    # automatically when first needed.
    WR_cont.set_vel(N, 0)
    WR_mc.v2pt_theory(WR_cont, N, WR)
    Steer.v2pt_theory(WR_mc, N, Frame)
    Frame_mc.v2pt_theory(WR_mc, N, Frame)
    Fork_mc.v2pt_theory(Steer, N, Fork)
    WF_mc.v2pt_theory(Steer, N, Fork)
    WF_cont.v2pt_theory(WF_mc, N, WF)

    # Sets the inertias of each body. Uses the inertia frame to construct the
    # inertia dyadics. Wheel inertias are only defined by principle moments of
    # inertia, and are in fact constant in the frame and fork reference frames;
    # it is for this reason that the orientations of the wheels does not need
    # to be defined. The frame and fork inertias are defined in the 'Temp'
    # frames which are fixed to the appropriate body frames; this is to allow
    # easier input of the reference values of the benchmark paper. Note that
    # due to slightly different orientations, the products of inertia need to
    # have their signs flipped; this is done later when entering the numerical
    # value.

    Frame_I = (inertia(TempFrame, Iframe11, Iframe22, Iframe33, 0, 0, Iframe31), Frame_mc)
    Fork_I = (inertia(TempFork, Ifork11, Ifork22, Ifork33, 0, 0, Ifork31), Fork_mc)
    WR_I = (inertia(Frame, Iwr11, Iwr22, Iwr11), WR_mc)
    WF_I = (inertia(Fork, Iwf11, Iwf22, Iwf11), WF_mc)

    # Declaration of the RigidBody containers. ::

    BodyFrame = RigidBody('BodyFrame', Frame_mc, Frame, mframe, Frame_I)
    BodyFork = RigidBody('BodyFork', Fork_mc, Fork, mfork, Fork_I)
    BodyWR = RigidBody('BodyWR', WR_mc, WR, mwr, WR_I)
    BodyWF = RigidBody('BodyWF', WF_mc, WF, mwf, WF_I)

    # The kinematic differential equations; they are defined quite simply. Each
    # entry in this list is equal to zero.
    kd = [q1d - u1, q2d - u2, q4d - u4, q5d - u5]

    # The nonholonomic constraints are the velocity of the front wheel contact
    # point dotted into the X, Y, and Z directions; the yaw frame is used as it
    # is "closer" to the front wheel (1 less DCM connecting them). These
    # constraints force the velocity of the front wheel contact point to be 0
    # in the inertial frame; the X and Y direction constraints enforce a
    # "no-slip" condition, and the Z direction constraint forces the front
    # wheel contact point to not move away from the ground frame, essentially
    # replicating the holonomic constraint which does not allow the frame pitch
    # to change in an invalid fashion.

    conlist_speed = [WF_cont.vel(N) & Y.x, WF_cont.vel(N) & Y.y, WF_cont.vel(N) & Y.z]

    # The holonomic constraint is that the position from the rear wheel contact
    # point to the front wheel contact point when dotted into the
    # normal-to-ground plane direction must be zero; effectively that the front
    # and rear wheel contact points are always touching the ground plane. This
    # is actually not part of the dynamic equations, but instead is necessary
    # for the lineraization process.

    conlist_coord = [WF_cont.pos_from(WR_cont) & Y.z]

    # The force list; each body has the appropriate gravitational force applied
    # at its mass center.
    # T2 : roll torque, between Newtonian frame and rear frame
    # T3 : rear wheel torque, between rear wheel and rear frame
    # T5 : steer torque, between rear frame and front frame
    T2, T3, T5 = dynamicsymbols('T2 T3 T5')
    Tf = friction_torque_fun(u5)

    FL = [(Frame_mc, -mframe * g * Y.z),
        (Fork_mc, -mfork * g * Y.z),
        (WF_mc, -mwf * g * Y.z),
        (WR_mc, -mwr * g * Y.z),
        (Frame, T2*Y.x - T3*R.y - (T5 + Tf)*Frame.x),
        (WR, T3*R.y),
        (Fork, (T5 + Tf)*Frame.x)]
    BL = [BodyFrame, BodyFork, BodyWR, BodyWF]

    # The N frame is the inertial frame, coordinates are supplied in the order
    # of independent, dependent coordinates, as are the speeds. The kinematic
    # differential equation are also entered here.  Here the dependent speeds
    # are specified, in the same order they were provided in earlier, along
    # with the non-holonomic constraints.  The dependent coordinate is also
    # provided, with the holonomic constraint.  Again, this is only provided
    # for the linearization process.

    KM = KanesMethod(
        N,
        q_ind=[q1, q2, q5],
        q_dependent=[q4],
        configuration_constraints=conlist_coord,
        u_ind=[u2, u3, u5],
        u_dependent=[u1, u4, u6],
        velocity_constraints=conlist_speed,
        kd_eqs=kd,
        constraint_solver="CRAMER")
    (fr, frstar) = KM.kanes_equations(BL, FL)

    # This is the start of entering in the numerical values from the benchmark
    # paper to validate the eigen values of the linearized equations from this
    # model to the reference eigen values. Look at the aforementioned paper for
    # more information. Some of these are intermediate values, used to
    # transform values from the paper into the coordinate systems used in this
    # model.
    w, r_F, r_R, c, lamb = symbols('w, r_F, r_R, c, lambda')           # Geometrical properties
    I_Rxx, I_Bxx, I_Hxx, I_Fxx = symbols('I_Rxx, I_Bxx, I_Hxx, I_Fxx') # Mass moment of inertia
    I_Bzz, I_Hzz = symbols('I_Bzz, I_Hzz')                             # Mass moment of inertia
    I_Ryy, I_Fyy = symbols('I_Ryy, I_Fyy')                             # Mass moment of inertia
    I_Byy, I_Hyy = symbols('I_Byy, I_Hyy')                             # Mass moment of inertia
    I_Bxz, I_Hxz = symbols('I_Bxz, I_Hxz')                             # Mass products of inertia  I_Bzz + I_Hzz
    m_R, m_B, m_H, m_F = symbols('m_R, m_B, m_H, m_F')                 # Mass
    x_B, x_H = symbols('x_B, x_H')                                     # center of mass x coordinate
    z_B, z_H = symbols('z_B, z_H')                                     # center of mass z coordinate

    meijaard_par_repl = {
    w       : 1.036, # [m]
    c       : 0.0803, # [m]
    lamb    : (18.2)*(pi/180), # [rad]
    g       : 9.81, # [m/(s**2)]

    r_R     : 0.3498, # [m]
    m_R     : 10.12, # [kg]
    I_Rxx   : 0.1040, # [kg*(m**2)]
    I_Ryy   : 0.1641, # [kg*(m**2)]

    x_B     : 0.462, # [m]
    z_B     : -0.698, # [m]
    m_B     : 20.9, # [kg]
    I_Bxx   : 1.64, # [kg*(m**2)]
    I_Bzz   : 1.94, # [kg*(m**2)]
    I_Bxz   : 0.654, # [kg*(m**2)]

    x_H     : 0.944, # [m]
    z_H     : -0.595, # [m]
    m_H     : 0.6, # [kg]
    I_Hxx   : 0.00980, # 0.0980, # [kg*(m**2)] # corrected:Bianchi Pista
    I_Hzz   : 0.00396, # 0.0396, # [kg*(m**2)] # corrected:Bianchi Pista
    I_Hxz   : -0.00044, # -0.0044, # [kg*(m**2)] # corrected:Bianchi Pista

    r_F     : 0.3498, # [m]
    m_F     : 1.780, # [kg]
    I_Fxx   : 0.0644, # [kg*(m**2)]
    I_Fyy   : 0.1289, # [kg*(m**2)]
    }

    val_dict = meijaard2kane(meijaard_par_repl)

    # The symbol 'v' is the forward speed of the bicycle (a concept which only 
    # makes sense in the upright, static equilibrium case?). 
    v = symbols('v')
    equilib_point = {
        u1.diff(): 0,
        u2.diff(): 0,
        u3.diff(): 0,
        u4.diff(): 0,
        u5.diff(): 0,
        u6.diff(): 0,
        u1: 0,
        u2: 0,
        u3: v / r_R.xreplace(meijaard_par_repl),
        u4: 0,
        u5: 0,
        u6: v / r_F.xreplace(meijaard_par_repl),
        q1: 0,
        q2: 0,
        q4: 0,
        q5: 0,
        T2: 0,
        T3: 0,
        T5: 0,
    }

    # Linearizes the forcing vector; the equations are set up as MM udot =
    # forcing, where MM is the mass matrix, udot is the vector representing the
    # time derivatives of the generalized speeds, and forcing is a vector which
    # contains both external forcing terms and internal forcing terms, such as
    # centripital or coriolis forces.  This actually returns a matrix with as
    # many rows as *total* coordinates and speeds, but only as many columns as
    # independent coordinates and speeds.
    print(1)
    A, B, r = KM.linearize(
        A_and_B=True,
        # Operating points for the accelerations are required for the
        # linearizer to eliminate u' terms showing up in the coefficient
        # matrices.
        op_point=equilib_point,
        linear_solver="CRAMER",
    )
    # As mentioned above, the size of the linearized forcing terms is expanded
    # to include both q's and u's, so the mass matrix must have this done as
    # well.  This will likely be changed to be part of the linearized process,
    # for future reference.
    print(2)
    # with open("solABr", "wb") as outf:
    #     dill.dump((A, B, r), outf)

    # with open("solABr","rb") as inf:
    #     solABr = dill.load(inf)
    # A, B, r = solABr

    A_s = A.xreplace(val_dict)
    B_s = B.xreplace(val_dict)

    A_s = A_s.evalf()
    B_s = B_s.evalf()

    # Finally, we construct an "A" matrix for the form xdot = A x (x being the
    # state vector, although in this case, the sizes are a little off). The
    # following line extracts only the minimum entries required for eigenvalue
    # analysis, which correspond to rows and columns for lean, steer, lean
    # rate, and steer rate.
    A_cw = A_s.extract([1, 2, 3, 5], [1, 2, 3, 5])
    B_cw = B_s.extract([1, 2, 3, 5], [0,2])

    # with open("solAB_cw", "wb") as outf:
    #     dill.dump((A_cw, B_cw), outf)

    # with open("solAB_cw","rb") as inf:
    #     solAB_cw = dill.load(inf)
    # A_cw,B_cw = solAB_cw          

    ss_cw_model = {
        "plant": {
            "A": lambdify(v, A_cw, 'numpy'),
            "B": lambdify((), B_cw, 'numpy')
        }
    }

    return ss_cw_model

def get_viscous_friction_fun(friction_coef):
    def viscous_friction_fun(steer_rate):
        return friction_coef*steer_rate
    return viscous_friction_fun

def get_sigmoid_friction_fun(friction_coef):
    def sigmoid_friction_fun(steer_rate):
        return friction_coef*tanh(10*steer_rate)
    return sigmoid_friction_fun

friction_functions = {
    # "viscous":get_viscous_friction_fun,
    "sigmoid":get_sigmoid_friction_fun,
}

for type, func in friction_functions.items():
    for fric_coef in linspace(-0.04,-0.09,6):
        ss_cw_friction = create_ss_cw_with_friction(func(fric_coef))
        with open(f"ss_cw_friction{fric_coef}_"+type, "wb") as outf:
            dill.dump(ss_cw_friction, outf)
        print(fric_coef)
    print(type)
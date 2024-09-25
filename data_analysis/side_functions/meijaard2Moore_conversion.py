'''
___[maijaard2Moore_conversion.py]___
Almost all scripts here are based on the variables
of the meijaard papar. At some point I was looking
into a different set of parameters based of a 
Moore paper. This would then allow the inclusion of 
friction.
[https://pydy.readthedocs.io/en/stable/examples/carvallo-whipple.html]
However, I eventually did get that model to work. 
But it felt bad to throw this conversion away.
'''

import sympy as sm
##--[define static symbols
#--Meijaard
g = sm.symbols('g')
w, r_F, r_R, c, lamb = sm.symbols('w, r_F, r_R, c, lambda')           # Geometrical properties
I_Rxx, I_Bxx, I_Hxx, I_Fxx = sm.symbols('I_Rxx, I_Bxx, I_Hxx, I_Fxx') # Mass moment of inertia
I_Bzz, I_Hzz = sm.symbols('I_Bzz, I_Hzz')                             # Mass moment of inertia
I_Ryy, I_Fyy = sm.symbols('I_Ryy, I_Fyy')                             # Mass moment of inertia
I_Byy, I_Hyy = sm.symbols('I_Byy, I_Hyy')                             # Mass moment of inertia
I_Bxz, I_Hxz = sm.symbols('I_Bxz, I_Hxz')                             # Mass products of inertia  I_Bzz + I_Hzz
m_R, m_B, m_H, m_F = sm.symbols('m_R, m_B, m_H, m_F')                 # Mass
x_B, x_H = sm.symbols('x_B, x_H')                                     # center of mass x coordinate
z_B, z_H = sm.symbols('z_B, z_H')                                     # center of mass z coordinate

#--Moore
rf, rr = sm.symbols('rf rr')
d1, d2, d3 = sm.symbols('d1 d2 d3')
l1, l2, l3, l4 = sm.symbols('l1 l2 l3 l4')
mc, md, me, mf = sm.symbols('mc md me mf')
ic11, ic22, ic33, ic31 = sm.symbols('ic11 ic22 ic33 ic31')
id11, id22 = sm.symbols('id11 id22')
ie11, ie22, ie33, ie31 = sm.symbols('ie11 ie22 ie33 ie31')
if11, if22 = sm.symbols('if11 if22')

##--[define Conversion function
def meijaard2moore(meijaard_values:dict):
    # Conversion to Moore2012
    rf_ = r_F
    rr_ = r_R
    d1_ = sm.cos(lamb) * (c + w - r_R*sm.tan(lamb))
    d3_ = -sm.cos(lamb) * (c - r_F*sm.tan(lamb))
    d2_ = (r_R + d1_*sm.sin(lamb) - r_F + d3_*sm.sin(lamb))/sm.cos(lamb)

    l1_ = x_B*sm.cos(lamb) - z_B*sm.sin(lamb) - r_R*sm.sin(lamb)
    l2_ = x_B*sm.sin(lamb) + z_B*sm.cos(lamb) + r_R*sm.cos(lamb)
    l4_ = (z_H + r_F)*sm.cos(lamb) + (x_H - w)*sm.sin(lamb)
    l3_ = (x_H - w - l4_*sm.sin(lamb))/sm.cos(lamb) 

    mc_ = m_B
    md_ = m_R
    me_ = m_H
    mf_ = m_F

    id11_ = I_Rxx
    id22_ = I_Ryy
    if11_ = I_Fxx
    if22_ = I_Fyy

    R = sm.rot_axis2(lamb)
    I_B_ = sm.Matrix([[I_Bxx, 0    , I_Bxz],
                    [0    , I_Byy, 0    ],
                    [I_Bxz, 0    , I_Bzz]])
    I_H_ = sm.Matrix([[I_Hxx, 0    , I_Hxz],
                    [0    , I_Hyy, 0    ],
                    [I_Hxz, 0    , I_Hzz]])

    I_C_ = R*I_B_*R.T
    I_E_ = R*I_H_*R.T

    ic11_  = I_C_[0,0]
    ic22_  = I_C_[1,1]
    ic31_  = I_C_[2,0]
    ic33_  = I_C_[2,2]
    ie11_  = I_E_[0,0]
    ie22_  = I_E_[1,1]
    ie31_  = I_E_[2,0]
    ie33_  = I_E_[2,2]
    
    converted_constants = {
        rf: rf_.xreplace(meijaard_values),
        rr: rr_.xreplace(meijaard_values),
        d1: d1_.xreplace(meijaard_values).evalf(),
        d3: d3_.xreplace(meijaard_values).evalf(),
        d2: d2_.xreplace(meijaard_values).evalf(),
        l1: l1_.xreplace(meijaard_values).evalf(),
        l2: l2_.xreplace(meijaard_values).evalf(),
        l4: l4_.xreplace(meijaard_values).evalf(),
        l3: l3_.xreplace(meijaard_values).evalf(),
        mc: mc_.xreplace(meijaard_values),
        md: md_.xreplace(meijaard_values),
        me: me_.xreplace(meijaard_values),
        mf: mf_.xreplace(meijaard_values),
        id11: id11_.xreplace(meijaard_values),
        id22: id22_.xreplace(meijaard_values),
        if11: if11_.xreplace(meijaard_values),
        if22: if22_.xreplace(meijaard_values),
        ic11: ic11_.xreplace(meijaard_values).evalf(),
        ic22: ic22_.xreplace(meijaard_values),
        ic31: ic31_.xreplace(meijaard_values).evalf(),
        ic33: ic33_.xreplace(meijaard_values).evalf(),
        ie11: ie11_.xreplace(meijaard_values).evalf(),
        ie22: ie22_.xreplace(meijaard_values),
        ie31: ie31_.xreplace(meijaard_values).evalf(),
        ie33: ie33_.xreplace(meijaard_values).evalf(),
        g: g.xreplace(meijaard_values)
    }

    return converted_constants





# ##--Test
# # Meijaard2007
# meijaard_par_repl = {
#     w      : 1.02,
#     c      : 0.08,
#     lamb   : sm.pi/10,
#     g      : 9.81,
#     v      : 5,
#     r_R    : 0.3,
#     m_R    : 2,
#     I_Rxx  : 0.0603,
#     I_Ryy  : 0.12,
#     x_B    : 0.3,
#     z_B    : -0.9,
#     m_B    : 85,
#     I_Bxx  : 9.2,
#     I_Byy  : 11,
#     I_Bzz  : 2.8,
#     I_Bxz  : 2.4,
#     x_H    : 0.9,
#     z_H    : -0.7,
#     m_H    : 4,
#     I_Hxx  : 0.05892,
#     I_Hyy  : 0.06,
#     I_Hzz  : 0.00708,
#     I_Hxz  : -0.00756,
#     r_F    : 0.35,
#     m_F    : 3,
#     I_Fxx  : 0.1405,
#     I_Fyy  : 0.28
# }

# moore_par_repl = {
#    rf: 0.35,
#    rr: 0.3,
#    d1: 0.9534570696121849,
#    d3: 0.03207142672761929,
#    d2: 0.2676445084476887,
#    l1: 0.4707271515135145,
#    l2: -0.47792881146460797,
#    l4: -0.3699518200282974,
#    l3: -0.00597083392418685,
#    mc: 85.0,
#    md: 2.0,
#    me: 4.0,
#    mf: 3.0,
#    id11: 0.0603,
#    id22: 0.12,
#    if11: 0.1405,
#    if22: 0.28,
#    ic11: 7.178169776497895,
#    ic22: 11.0,
#    ic31: 3.8225535938357873,
#    ic33: 4.821830223502103,
#    ie11: 0.05841337700152972,
#    ie22: 0.06,
#    ie31: 0.009119225261946298,
#    ie33: 0.007586622998470264,
#    g: 9.81
# }

# all_pars = sm.Matrix([
#    rf,
#    rr,
#    d1,
#    d3,
#    d2,
#    l1,
#    l2,
#    l4,
#    l3,
#    mc,
#    md,
#    me,
#    mf,
#    id11,
#    id22,
#    if11,
#    if22,
#    ic11,
#    ic22,
#    ic31,
#    ic33,
#    ie11,
#    ie22,
#    ie31,
#    ie33,
#    g
# ])

# meij2moore_par_repl = meijaard2moore(meijaard_par_repl)
# moore_par_val = all_pars.xreplace(moore_par_repl)
# converted_moore_par_val = all_pars.xreplace(meij2moore_par_repl)

# sm.pprint(moore_par_val-converted_moore_par_val)
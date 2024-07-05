import sympy as sm
from sympy.core.symbol import symbols
from sympy.core.numbers import pi
from sympy.functions.elementary.miscellaneous import sqrt
from sympy.functions.elementary.trigonometric import acos, sin, cos

##--[define static symbols
#--Meijaard
g = symbols('g')
w, r_F, r_R, c, lamb = symbols('w, r_F, r_R, c, lambda')           # Geometrical properties
I_Rxx, I_Bxx, I_Hxx, I_Fxx = symbols('I_Rxx, I_Bxx, I_Hxx, I_Fxx') # Mass moment of inertia
I_Bzz, I_Hzz = symbols('I_Bzz, I_Hzz')                             # Mass moment of inertia
I_Ryy, I_Fyy = symbols('I_Ryy, I_Fyy')                             # Mass moment of inertia
I_Byy, I_Hyy = symbols('I_Byy, I_Hyy')                             # Mass moment of inertia
I_Bxz, I_Hxz = symbols('I_Bxz, I_Hxz')                             # Mass products of inertia  I_Bzz + I_Hzz
m_R, m_B, m_H, m_F = symbols('m_R, m_B, m_H, m_F')                 # Mass
x_B, x_H = symbols('x_B, x_H')                                     # center of mass x coordinate
z_B, z_H = symbols('z_B, z_H')                                     # center of mass z coordinate

#--Kane3
WFrad, WRrad, htangle, forkoffset = symbols('WFrad WRrad htangle forkoffset')
forklength, framelength, forkcg1 = symbols('forklength framelength forkcg1')
forkcg3, framecg1, framecg3, Iwr11 = symbols('forkcg3 framecg1 framecg3 Iwr11')
Iwr22, Iwf11, Iwf22, Iframe11 = symbols('Iwr22 Iwf11 Iwf22 Iframe11')
Iframe22, Iframe33, Iframe31, Ifork11 = symbols('Iframe22 Iframe33 Iframe31 Ifork11')
Ifork22, Ifork33, Ifork31, g = symbols('Ifork22 Ifork33 Ifork31 g')
mframe, mfork, mwf, mwr = symbols('mframe mfork mwf mwr')

##--[define Conversion function
def meijaard2kane(meijaard_values:dict):
   PaperRadRear                    =  r_R
   PaperRadFront                   =  r_F
   HTA                             =  (pi/2 - lamb)
   TrailPaper                      =  c
   rake                            =  (-(TrailPaper*sin(HTA)-(PaperRadFront*cos(HTA))))
   PaperWb                         =  w
   PaperFrameCgX                   =  x_B
   PaperFrameCgZ                   =  -z_B
   PaperForkCgX                    =  x_H
   PaperForkCgZ                    =  -z_H
   rake                            =  (-(TrailPaper*sin(HTA)-(PaperRadFront*cos(HTA))))
   FrameLength                     =  (PaperWb*sin(HTA)-(rake-(PaperRadFront-PaperRadRear)*cos(HTA)))
   FrameCGNorm                     =  ((PaperFrameCgZ - PaperRadRear-(PaperFrameCgX/sin(HTA))*cos(HTA))*sin(HTA))
   FrameCGPar                      =  (PaperFrameCgX / sin(HTA) + (PaperFrameCgZ - PaperRadRear - PaperFrameCgX / sin(HTA) * cos(HTA)) * cos(HTA))
   tempa                           =  (PaperForkCgZ - PaperRadFront)
   tempb                           =  (PaperWb-PaperForkCgX)
   tempc                           =  (sqrt(tempa**2+tempb**2))
   PaperForkL                      =  (PaperWb*cos(HTA)-(PaperRadFront-PaperRadRear)*sin(HTA))
   ForkCGNorm                      =  (rake+(tempc * sin(pi/2-HTA-acos(tempa/tempc))))
   ForkCGPar                       =  (tempc * cos((pi/2-HTA)-acos(tempa/tempc))-PaperForkL)

   val_dict = {WFrad: PaperRadFront.xreplace(meijaard_values),
               WRrad: PaperRadRear.xreplace(meijaard_values),
               htangle: HTA.xreplace(meijaard_values).evalf(),
               forkoffset: rake.xreplace(meijaard_values).evalf(),
               forklength: PaperForkL.xreplace(meijaard_values).evalf(),
               framelength: FrameLength.xreplace(meijaard_values).evalf(),
               forkcg1: ForkCGPar.xreplace(meijaard_values).evalf(),
               forkcg3: ForkCGNorm.xreplace(meijaard_values).evalf(),
               framecg1: FrameCGNorm.xreplace(meijaard_values).evalf(),
               framecg3: FrameCGPar.xreplace(meijaard_values).evalf(),
               Iwr11: I_Rxx.xreplace(meijaard_values),
               Iwr22: I_Ryy.xreplace(meijaard_values),
               Iwf11: I_Fxx.xreplace(meijaard_values),
               Iwf22: I_Fyy.xreplace(meijaard_values),
               Ifork11: I_Hxx.xreplace(meijaard_values),
               Ifork22: I_Hyy.xreplace(meijaard_values),
               Ifork33: I_Hzz.xreplace(meijaard_values),
               Ifork31: -I_Hxz.xreplace(meijaard_values),
               Iframe11: I_Bxx.xreplace(meijaard_values),
               Iframe22: I_Byy.xreplace(meijaard_values),
               Iframe33: I_Bzz.xreplace(meijaard_values),
               Iframe31: -I_Bxz.xreplace(meijaard_values),
               mfork: m_H.xreplace(meijaard_values),
               mframe: m_B.xreplace(meijaard_values),
               mwf: m_F.xreplace(meijaard_values),
               mwr: m_R.xreplace(meijaard_values),
               g: g.xreplace(meijaard_values)}

   return val_dict





# ##--Test
# # Meijaard2007
# meijaard_par_repl = {
#     w      : 1.02,
#     c      : 0.08,
#     lamb   : sm.pi/10,
#     g      : 9.81,
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

# all_pars = sm.Matrix([
#    WFrad,
#    WRrad,
#    htangle,
#    forkoffset,
#    forklength,
#    framelength,
#    forkcg1,
#    forkcg3,
#    framecg1,
#    framecg3,
#    Iwr11,
#    Iwr22,
#    Iwf11,
#    Iwf22,
#    Ifork11,
#    Ifork22,
#    Ifork33,
#    Ifork31,
#    Iframe11,
#    Iframe22,
#    Iframe33,
#    Iframe31,
#    mfork,
#    mframe,
#    mwf,
#    mwr,
#    g
# ])

# meij2kane_par_repl = meijaard2kane(meijaard_par_repl)
# converted_moore_par_val = all_pars.xreplace(meij2kane_par_repl)
# sm.pprint(converted_moore_par_val)


#--------------------------------------------------------------------------------------------------
# #Sympy symb   : in Meijaard    :  Sympy value         <-->  Meij. symb : Meij. val
# PaperWb       : w              =  1.02                <-->      w      : 1.02
# TrailPaper    : c              =  0.08                <-->      c      : 0.08
# HTA           : 0.5*pi-lamb    =  (pi / 2 - pi / 10)  <-->      lamb   : sm.pi/10   <----- Caution
# g             : g              =  9.81                <-->      g      : 9.81
# v             : v              =  ...                 <-->      v      : 5
# PaperRadRear  : r_R            =  0.3                 <-->      r_R    : 0.3
# mwr           : m_R            =  2                   <-->      m_R    : 2
# Iwr11         : I_Rxx          =  0.0603              <-->      I_Rxx  : 0.0603
# Iwr22         : I_Ryy          =  0.12                <-->      I_Ryy  : 0.12
# PaperFrameCgX : x_B            =  0.3                 <-->      x_B    : 0.3
# PaperFrameCgZ : z_B            =  0.9                 <-->      z_B    : -0.9       <----- Caution
# mframe        : m_B            =  85                  <-->      m_B    : 85
# Iframe11      : I_Bxx          =  9.2                 <-->      I_Bxx  : 9.2
# Iframe22      : I_Byy          =  11                  <-->      I_Byy  : 11
# Iframe33      : I_Bzz          =  2.8                 <-->      I_Bzz  : 2.8
# Iframe31      : I_Bxz          =  -2.4                <-->      I_Bxz  : 2.4        <----- Caution
# PaperForkCgX  : x_H            =  0.9                 <-->      x_H    : 0.9
# PaperForkCgZ  : z_H            =  0.7                 <-->      z_H    : -0.7       <----- Caution
# mfork         : m_H            =  4                   <-->      m_H    : 4
# Ifork11       : I_Hxx          =  0.05892             <-->      I_Hxx  : 0.05892
# Ifork22       : I_Hyy          =  0.06                <-->      I_Hyy  : 0.06
# Ifork33       : I_Hzz          =  0.00708             <-->      I_Hzz  : 0.00708
# Ifork31       : I_Hxz          =  0.00756             <-->      I_Hxz  : -0.00756   <----- Caution
# PaperRadFront : r_F            =  0.35                <-->      r_F    : 0.35
# mwf           : m_F            =  3                   <-->      m_F    : 3
# Iwf11         : I_Fxx          =  0.1405              <-->      I_Fxx  : 0.1405
# Iwf22         : I_Fyy          =  0.28                <-->      I_Fyy  : 0.28
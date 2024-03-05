# To see if it will give 0=0 for the original equation
# But it gave a few nans and a zoo(?)

# sm.pprint(sm.simplify(phi_coef.xreplace(sol1[0])))
# sm.pprint(sm.simplify(delta_coef.xreplace(sol1[0])))
# sm.pprint(sm.simplify(dphi_coef.xreplace(sol1[0])))
# sm.pprint(sm.simplify(ddelta_coef.xreplace(sol1[0])))
# sm.pprint(sm.simplify(Tphi_coef.xreplace(sol1[0])))
# sm.pprint(sm.simplify(Tdelta_coef.xreplace(sol1[0])))

# Thus lets see if we look at the original equations
# But this gave all zeros. So deviding by B or multiplying by B did not mater... 
# WITH RESPECT TO THE NUMERATORs
# This leaves, either M_inv_det being the one that causes problems. Or the denominators in fraction

# phi_coef_t = sm.factor((A_r[2,0] - A[2,0]) / (B[2,1]*M_inv_det) - (A_r[3,0] - A[3,0]) / (B[3,1]*M_inv_det))
# delta_coef_t = sm.factor((A_r[2,1] - A[2,1]) / (B[2,1]*M_inv_det) - (A_r[3,1] - A[3,1]) / (B[3,1]*M_inv_det))
# dphi_coef_t = sm.factor((A_r[2,2] - A[2,2]) / (B[2,1]*M_inv_det) - (A_r[3,2] - A[3,2]) / (B[3,1]*M_inv_det))
# ddelta_coef_t = sm.factor((A_r[2,3] - A[2,3]) / (B[2,1]*M_inv_det) - (A_r[3,3] - A[3,3]) / (B[3,1]*M_inv_det))
# Tphi_coef_t = sm.factor((B_r[2,0] - B[2,0]) / (B[2,1]*M_inv_det) - (B_r[3,0] - B[3,0]) / (B[3,1]*M_inv_det))
# Tdelta_coef_t = sm.factor(B_r[2,1] / (B[2,1]*M_inv_det) - B_r[3,1] / (B[3,1]*M_inv_det))

# phi_coef_t, _ = sm.fraction(phi_coef_t)
# delta_coef_t, _ = sm.fraction(delta_coef_t)
# dphi_coef_t, _ = sm.fraction(dphi_coef_t)
# ddelta_coef_t, _ = sm.fraction(ddelta_coef_t)
# Tphi_coef_t, _ = sm.fraction(Tphi_coef_t)
# Tdelta_coef_t, _ = sm.fraction(Tdelta_coef_t)

# sm.pprint(sm.simplify((phi_coef + phi_coef_t)))
# sm.pprint(sm.simplify((delta_coef + delta_coef_t)))
# sm.pprint(sm.simplify((dphi_coef + dphi_coef_t)))
# sm.pprint(sm.simplify((ddelta_coef + ddelta_coef_t)))
# sm.pprint(sm.simplify((Tphi_coef + Tphi_coef_t)))
# sm.pprint(sm.simplify((Tdelta_coef + Tdelta_coef_t)))


# Could not be M_inv_det, as this is all in normal variables

# sm.pprint(M_inv_det)


# Than running the code and doing xreplace(sol1) without the removal of the fractions, gave tha nans

# phi_coef = sm.factor((A_r[2,0] - A[2,0]) * (B[3,1] * M_inv_det)  - (A_r[3,0] - A[3,0]) * (B[2,1] * M_inv_det))
# delta_coef = sm.factor((A_r[2,1] - A[2,1]) * (B[3,1] * M_inv_det) - (A_r[3,1] - A[3,1]) * (B[2,1] * M_inv_det))
# dphi_coef = sm.factor((A_r[2,2] - A[2,2]) * (B[3,1] * M_inv_det) - (A_r[3,2] - A[3,2]) * (B[2,1] * M_inv_det))
# ddelta_coef = sm.factor((A_r[2,3] - A[2,3]) * (B[3,1] * M_inv_det) - (A_r[3,3] - A[3,3]) * (B[2,1] * M_inv_det))
# Tphi_coef = sm.factor((B_r[2,0] - B[2,0]) * (B[3,1] * M_inv_det) - (B_r[3,0] - B[3,0]) * (B[2,1] * M_inv_det))
# Tdelta_coef = sm.factor(B_r[2,1] * (B[3,1] * M_inv_det) - B_r[3,1] * (B[2,1] * M_inv_det))

# sm.pprint(sm.simplify(phi_coef.xreplace(sol1[0])))        nan
# sm.pprint(sm.simplify(delta_coef.xreplace(sol1[0])))      nan
# sm.pprint(sm.simplify(dphi_coef.xreplace(sol1[0])))       0
# sm.pprint(sm.simplify(ddelta_coef.xreplace(sol1[0])))     nan
# sm.pprint(sm.simplify(Tphi_coef.xreplace(sol1[0])))       -1
# sm.pprint(sm.simplify(Tdelta_coef.xreplace(sol1[0])))     0



## Turned out that this [below] was causing the problems, so removing it gave new results.

# phi_coef, _ = sm.fraction(phi_coef)
# delta_coef, _ = sm.fraction(delta_coef)
# dphi_coef, _ = sm.fraction(dphi_coef)
# ddelta_coef, _ = sm.fraction(ddelta_coef)
# Tphi_coef, _ = sm.fraction(Tphi_coef)
# Tdelta_coef, _ = sm.fraction(Tdelta_coef)

## The new solution gave a nice 0=0 for the original equation
# phi_coef_t = (A_r[2,0] - A[2,0]) / B[2,1] - (A_r[3,0] - A[3,0]) / B[3,1]
# delta_coef_t = (A_r[2,1] - A[2,1]) / B[2,1] - (A_r[3,1] - A[3,1]) / B[3,1]
# dphi_coef_t = (A_r[2,2] - A[2,2]) / B[2,1] - (A_r[3,2] - A[3,2]) / B[3,1]
# ddelta_coef_t = (A_r[2,3] - A[2,3]) / B[2,1] - (A_r[3,3] - A[3,3]) / B[3,1]
# Tphi_coef_t = (B_r[2,0] - B[2,0]) / B[2,1] - (B_r[3,0] - B[3,0]) / B[3,1]
# Tdelta_coef_t = B_r[2,1] / B[2,1] - B_r[3,1] / B[3,1]

# sm.pprint(sm.simplify(phi_coef_t.xreplace(sol1[0]).xreplace(sol2[0])))
# sm.pprint(sm.simplify(delta_coef_t.xreplace(sol1[0]).xreplace(sol2[0])))
# sm.pprint(sm.simplify(dphi_coef_t.xreplace(sol1[0]).xreplace(sol2[0])))
# sm.pprint(sm.simplify(ddelta_coef_t.xreplace(sol1[0]).xreplace(sol2[0])))
# sm.pprint(sm.simplify(Tphi_coef_t.xreplace(sol1[0]).xreplace(sol2[0])))
# sm.pprint(sm.simplify(Tdelta_coef_t.xreplace(sol1[0]).xreplace(sol2[0]))) 


import sympy as sm
from sympy.abc import x, y, z, w
subs, cse_exp  = sm.cse(((w + x + y + z)*(w + y + z))/(w + x)**3)
for i in range(len(subs)):
    cse_exp[0] = cse_exp[0].xreplace({subs[-i][0]:subs[-i][1]})
# dictie = {subs[-1][0]:subs[-1][1]}
sm.pprint(cse_exp[0])

# subs, simp = sm.cse(delta_coef)
# sm.pprint(simp[0])
# hand_simp = (((subs[29][0] / subs[73][0]) - (subs[73][0] / subs[1][0]))*subs[74][0]
#     + ((subs[75][0] / subs[1][0]) - (subs[66][0] / subs[73][0]))*subs[76][0]
#     + ((subs[38][0] / subs[1][0]) - (subs[75][0] / subs[73][0]))*subs[72][0]
# )

# for i in range(len(subs)):
#     hand_simp = hand_simp.xreplace({subs[-i][0]:subs[-i][1]})

# sm.pprint(len(hand_simp.free_symbols))
# subs1, simp1 = sm.cse(hand_simp)

# sm.pprint(subs1)
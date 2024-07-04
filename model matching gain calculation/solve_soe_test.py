import sympy as sm

x,y,z,a = sm.symbols('x,y,z,a')

expr1 = x - a
expr2 = y + a
expr3 = x + y - z - a

sol = sm.solve([expr1,expr2,expr3], [x,y,z], dict=True)
sm.pprint(sol)
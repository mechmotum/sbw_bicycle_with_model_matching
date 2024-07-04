from sympy.abc import x, y, z
from sympy import solve, pprint
sol = solve([x - y, z - x], [x,y], dict=True)
pprint(sol)
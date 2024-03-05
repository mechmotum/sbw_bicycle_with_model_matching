import sympy as sm
import numpy as np

a,b,c,d,v = sm.symbols('a,b,c,d,v')

mat = sm.Matrix([[a,b],[c,d]])

repl = {
    a:1,
    b:2,
    c:3,
    d:4
}

mat = mat.xreplace(repl)

matfun = sm.lambdify([],mat,'numpy')
print(matfun())
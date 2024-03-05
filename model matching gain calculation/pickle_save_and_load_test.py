import sympy as sm
import pickle as pkl

a,b,c = sm.symbols('a,b,c')
r,q = sm.symbols('r,q')

y = [a**5/(b*sm.cos(c)) + sm.sin(sm.sqrt(a))/5, a**2 + b**2]
sm.pprint(y)

with open("y.txt", "wb") as outf:
    pkl.dump(y, outf)

with open("y.txt","rb") as inf:
    q,r = pkl.load(inf)

sm.pprint(q)
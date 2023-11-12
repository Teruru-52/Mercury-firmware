clear all

syms xG yG zG xX yX zX ddxG ddyG ddzG m dhGx dhGy dhGz g

pG = [xG; yG; zG];
pX = [xX; yX; zX];
ddpG = [ddxG; ddyG; ddzG];
dhG = [dhGx; dhGy; dhGz];

term1 = cross((pG-pX), m*(ddpG+[0; 0; g]))
eq = cross([0 0 g], (term1+dhG))

eq = eq/g;
eq(1)
eq(2)
eq(3)
= Compliance Control

This chapter is continued from regulation control in Chapter2. Denote $Delta x_(d e) = vec(Delta p _(d e), Delta phi_(d e))$ and rewrite regulation control law
$
    gamma = H^(-top)(phi_e) K_P Delta x_(d e)
$
$
  K_P = mat(
    K_(P p), O;
    O, K_(P o)
  ) , quad H = mat(
    I, O;
    O, T
  )
$
which leads to closed-loop dynamic equation
$
  B dot.double(q)  + C dot(q) + (F + K_D) dot(q) quad = J^(top)(gamma -h)
$
The steady state $dot(q) = dot.double(q)= 0$ of the system is 
$
  gamma = h => H^(-top) (phi_e)K_P Delta x_(d e)= h \
  => Delta x_(d e)= K_P^(-1) H^(top) (phi_e) h
$
It shows that at steady state the manipulator, under a proportional control action on the position and orientation error, behaves as a generalized spring in respect of the force and moment $h$. Separating above into 
$
    Delta p_(d e) = K_(P p)^(-1) f, quad Delta phi_(d e) = K_(P o)^(-1) T^(top) (phi_e) mu
$ 
Considering a frictionless and elastically compliant environment
$
  f = K_f (p_e - p_o) \
  => Delta p_(d e) = K_(P p) ^(-1) K_f (p_e - p_o)
$
thus, at steady state, the end-effector position and the contact force are given by
$
  p_(e, oo) quad = (I + K_(P p)^(-1) K_f)^(-1) (p_d + K_(P p)^(-1) K_f p_o)\
  f_(oo) =(I + K_f K_(P p)^(-1))^(-1) K_f (p_d - p_o)
$

= Impedance Control

Control law
$
  tau = B(q) alpha +C(q,dot(q)) dot(q) + F dot(q)  + g(q) 
$
with $h != 0$ leads to
$
  dot.double(q) = alpha -B^(-1) (q) J^(top) (q) h
$
Choice of $alpha$
$
  alpha_p =  dot.double(p)_d + K_(D p) Delta dot(p) _(d e)+ K_(P p) Delta p _(d e)\
  alpha_o = T(phi_e) (dot.double(phi)_d + K_(D o) Delta dot(phi) _(d e) + K_(P o) Delta phi_(d e)) + dot(T) (phi_e, dot(phi)_e) dot(phi)_e
$


When considering the end-effector moving in free space $(h=0)$ with disturbance $(d !=0$), the error dynamic of pure *impedance control* is 
$
  Delta dot.double(p)_(d e) + K_(M p)^(-1) K_(D p) Delta dot(p) _(d e) + K^(-1)_(M p) K_(P p) Delta p_(d e) = d
$
while the error dynamic of *impedance control with inner motion control* is
$
   Delta dot.double(p)_(d e)+ K_(D p) Delta dot(p)_(d e)+ K_(P p) Delta p_( d e)= d
$

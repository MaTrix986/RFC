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
Previous choice of $alpha$
$
  alpha_p =  dot.double(p)_d + K_(D p) Delta dot(p) _(d e)+ K_(P p) Delta p _(d e)\
  alpha_o = T(phi_e) (dot.double(phi)_d + K_(D o) Delta dot(phi) _(d e) + K_(P o) Delta phi_(d e)) + dot(T) (phi_e, dot(phi)_e) dot(phi)_e
$
leads to 
$
  Delta dot.double(x)_(d e) +K_D Delta dot(x)_(d e) + K_P Delta x_(d e)= H ^(-1)(phi_e) J(q) B^(-1)(q) J^(top) (q) h
$
Denote by $M = J^(-top) B J^(-1) H$, we have
$
  M Delta dot.double(x)_(d e) + M K_D Delta dot(x)_(d e) + M K_P Delta x_(d e)=  h 
$
The presence of $M$ makes the system coupled and nonlinear. If it is wished to keep linearity and decoupling during the interaction with the environment ($h != 0$), Choose
$
  tau  = B alpha + C dot(q) + F dot(q)+ g(q) + J^(top) h
$
with $alpha$
$
  a_p = dot.double(p)_d + K_(M p)^(-1) (K_(D p) Delta dot(p) _(d e) + K_(P p) Delta p_(d e) - f)\
  a_o = T(phi_e) (dot.double(phi)_d + K_(M o)^(-1) (K_(D o) Delta dot(phi)_(d e) + K_(P o) Delta phi_(d e) - T^(top) (phi_e) mu ))+dot(T)(phi_e ,dot(phi)_e)dot(phi)_e
$
with $K_(M p), K_(M o)$ position definite matrix gain. The term $T^(top) mu$ is aimed to preserve energy:
$
  W= mu ^(top) omega_e = mu^(top) T phi_e =(T^(top) mu)^(top) phi_e
$
Then we have 
$
  K_(M p) Delta dot.double(p)_(d e) + K_(D p) Delta dot(p)_(d e) + K_(P p) Delta p_(d e) =f\
  K_(M o) Delta dot.double(phi)_(d e) + K_(D o)Delta dot(phi)_( d e) + K_(P o) Delta phi_(d e) = T^(top) (phi_(e)) mu
$
Notice that tracking accuracy while constraint-free ($h=0$) and compliance preformace while interacting with environment ($h !=0$) are both determinated by $K_(M *), K_(D *), K_(P *)$. 

$
  K_M Delta dot.double(x)_(d e)+ K_D Delta dot(x)_(d e) + K_(P) Delta x_(d e) = H^(top)(phi_e) h + K_M H^(-1)(phi_e) d 
$

Therefore we introduce *inner motion control*. Considering complianct frame $Sigma_c$, translation part of it satifies dynamic system
$
   M_p Delta dot.double(p)_(d c) + D_p Delta dot(p)_(d c) + K_p Delta p_( d c) =f
$
we can cumpute $p_c$ in process via integration by initial value $p_c (0) = p_d (0)$. Then use motion control to track $p_c$ 
$
  a_p = dot.double(p)_c + K_(D p) Delta dot(p)_(c e) + K_(P p) Delta p_(c e)
$


When considering the end-effector moving in free space $(h=0)$ with disturbance $(d !=0$), the error dynamic of pure *impedance control* is 
$
  Delta dot.double(p)_(d e) + K_(M p)^(-1) K_(D p) Delta dot(p) _(d e) + K^(-1)_(M p) K_(P p) Delta p_(d e) = d
$
while the error dynamic of *impedance control with inner motion control* is
$
   Delta dot.double(p)_(d e)+ K_(D p) Delta dot(p)_(d e)+ K_(P p) Delta p_( d e)= d, quad (p_c = p_d )
$




== Geometric consistency of translation

geometric consistency is consistency between end-effector behavior and external force 

== Euler angle

== Axis/Angle 

== Quaternion

Extracing relative quaternion ${eta_(d c), ""^(c) epsilon_(d c)}$ from relative rotation $""^(c)R_d = R_c^(top) R_d$. Let relative rotation angle & aixs be $theta_(d c), u$, then
$ 
  eta_(d c) = cos theta.alt_(d c) / 2 , quad epsilon.alt_(d c) = sin theta.alt_(d c) / 2 u 
$
3.3 gives potenital energy and elastical moment
$ cal(U)_o = 2^c epsilon.alt_(d c)^(top) K_o^c epsilon.alt_(d c) ,\
""^c mu_E = 2 E^(top) (eta_(d c) , zws^c epsilon.alt_(d c)) K_o zws^c epsilon.alt_(d c) . $





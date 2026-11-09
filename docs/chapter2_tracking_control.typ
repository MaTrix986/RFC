= Tracking control

viscous friciotn $tau_f = F dot(q) $ Dynamic model
$
  B(q) dot.double(q) + C(q,dot(q))dot(q) + F dot(q) + g(q) = tau - J^(T)(q) h
$
According to this *dynamic model-based compensation*, the joint torques can be chosen as
$
  tau = B (q) alpha + C(q, dot(q)) dot(q) + F dot(q) + g(q)
$
$
  => quad dot.double(q) = alpha
$
The quantity $alpha$ represents a *resolved acceleration* in terms of joint variables. In task space, consider
$
  dot(v)_e = J(q)  dot.double(q) + dot(J)(q, dot(q)) dot(q)\
  => dot.double(q) = J^(-1)(q ) (dot(v)_e - dot(J) (q, dot(q)) dot(q) )
$
new control input $alpha$ in can be chosen as
$
  alpha =  J^(-1)(q ) (dot(v)_e - dot(J) (q, dot(q)) dot(q) )
$
$dot.double(q) = alpha$ leads to
$
  dot(v)_e =a => cases(
    dot.double(p)_e = a_p,
    dot(omega)_e = a_o
  )
$
We have already used the Jaccobian to transform the problem of joint space control into taskspace control. Position control start with $(p_d , dot(p)_d, dot.double(p)_d; p_e, dot(p)_e)$. A position error between the desired and the actual end-effector position can be defined as
$
   Delta p _(d e) =p_d - p_e
$
Then, the resolved linear acceleration can be chosen as
$
  a_p = dot.double(p)_d + K_(D_p) Delta dot(p) _(d e) + K_(P_p) Delta p_(d e)
$
which leads to 
$
   Delta dot.double(p)_(d e)+ K_(D_p) Delta dot(p) _(d e) + K_(P_p) Delta p_(d e) =0 
$

In pratical, angular controlling start with $(R_d, omega_d , dot(omega)_d; R_e, omega_e)$. In terms of choice of angles error, we have three different control strategys. 


== Classical Euler angles

$
  Delta phi_(d e) = phi_d - phi_e 
$
where $phi_d$ and $phi_e$ are the set of Euler angles that can be extracted respectively from the rotation matrices $R_d$ and $R_e$ describing the orientation of $Sigma_d$ and $Sigma_e$. Jaccobian between $omega_e$ and $dot(phi)_e$:
$
  omega_e = T(phi_e) dot(phi)_e\
  => dot(omega)_e = T(phi_e) dot.double(phi_e)+ dot(T)(phi_e, dot(phi)_e) dot(phi)_e
$
We obtain $dot(phi)_d$ form $phi_d$ by $dot(phi)_d = T^(-1)(phi_d) omega_d$. And similarly,
$
   dot(omega)_d = T(phi_d) dot.double(phi)_d+ dot(T)(phi_d, dot(phi)_d) dot(phi)_d\
   => dot.double(phi)_d = T^(-1)(phi_d) (dot(omega)_d - dot(T)(phi_d, dot(phi)_d) dot(phi)_d)
$
The resolved angular acceleration based on the Euler angles error can be chosen as
$
   a_o = T(phi_e) (dot.double(phi)_d + K_(D o) Delta dot(phi)_(d e) + K_(P o) Delta phi_(d e))+ dot(T)(phi_e, dot(phi)_e) dot(phi)_e
$
which leads to
$
  Delta dot.double(phi) _(d e) + K _(D o) Delta dot(phi)_(d e)+ K_(P o ) Delta phi_(d e) =0
$

== Alternative Euler angles

The above Euler angles error becomes ill-conditioned when the actual endeffector orientation 'Pe becomes close to a representation singularity. 
$
  ""^(e) R_e = R_e^(top) R_d \
  ""^(e) dot(R)_d = S(Delta^(e)omega_(d e))^(e) R_d
$
Let $phi_(d e)$ denote the set of Euler angles that can be extracted from $""^(e) R_d$. We can derive that
$
  Delta^(e) omega_(d e) = T(phi_(d e))dot(phi)_(d e)\
  => dot(omega)_e = dot(omega)_d - R_e T(phi_(d e)) dot.double(phi)_(d e) - R_e dot(T)(phi_(d e), dot(phi)_(d e)) dot(phi)_(d e)
$
The resolved angular acceleration can be chosen as
$
  a_o =  dot(omega)_d + R_e T(phi_(d e)) (K_(D o) dot(phi)_(d e) + K_(P o) phi_(d e)) - R_e dot(T)(phi_(d e), dot(phi)_(d e)) dot(phi)_(d e)
$
which leads to
$
  dot.double(phi)_(d e) + K_(D o)  dot(phi)_(d e) + K_(P o) phi_(d e) =0
$
By adopting a representation $phi_(d e) = [alpha_(d e), beta_(d e), gamma_(d e)]^(top)$ for which $T(0)$ is nonsingular, representation singularities occur only for large orientation errors.




== Angle/axis

== Quaternion


#image("/assets/image.png")


== Redundancy Resolution 

Consider right pseudo-inverse of $J$ $(6 times n)$ weighted by the positive definite $(n times  n)$ matrix $W$ 
$
  J^(dagger) = W^(-1) J^(top) (J W ^(-1) J^(top))^(-1)
$
we can replace $alpha =  J^(-1)(q ) (dot(v)_e - dot(J) (q, dot(q)) dot(q) )$ with
$
  dot.double(q) = alpha =  J^(dagger)(q ) (a - dot(J) (q, dot(q)) dot(q) ) + alpha_n
$
$alpha_n$ denotes a joint acceleration vector lying in the null space of $J$ which is available for redundancy resolution. Premultiplying both sides of above by $J$, 
$
  J dot.double(q) = a - dot(J)(q , dot(q)) dot(q)
$
We choose $W = B$ to make that external force and moment produce no null space joint accelerations. i.e.
$
  dot.double(q)_e = - B ^(-1) (q) J^(top)(q) h \
  => dot.double(q)_(e n )= - (I - J ^(dagger) J) B ^(-1) (q) J^(top)(q) h = 0
$
So originally, when consider external force and moment, the control law should be 
$
  dot.double(q) = J^(dagger) (a - dot(J) dot(q)) + alpha_n - B^(-1) J^(T) h\
  => N dot.double(q) = alpha_n - N B^(-1) J^(T) h
$
we choose $W= B$ to make $N B^(-1) J^(T) h =0 $. Consider null space velocity error where $beta$ is a joint velocity vector which is available for redundancy resolution.
$
  e_n = (I - J^(dagger) J) (beta- dot(q))\
  dot(e)_n = N (dot(beta) - alpha_(n)) - (dot(J)^(dagger) J + J^(dagger) dot(J)) (beta- dot(q)), quad (N J^(dagger) = 0)
$
The goal is to make en asymptotically converge to zero, hence we need to find a proper $alpha_(n)$. Consider
$
  cal(V) = 1/2 e_n^(top) B e_n
$ 
let $r = beta - dot(q)$
$
  dot(cal(V)) =&  1/2 e_n^(top) dot(B) e_n + e_n^(top) B dot(e)_n \
  =& 1/2 e_n^(top) dot(B) e_n + e_n^(top) B N (dot(beta) - alpha_n) - e_n^(top) B (dot(J)^(dagger) J + J^(dagger) dot(J)) r \
  =& 1/2 e_n^(top) dot(B) e_n + e_n^(top) B (dot(beta) - alpha_n - dot(J)^(dagger ) J r)
$
Choosing
$
  alpha_n = N (dot(beta) - dot(J)^(dagger ) J r  + B^(-1)(K_n e_n + C e_n))  
$
By $e_n^(top) B N = e_n^(top) B $, 
$ 
  e_n^(⊺) B alpha_n = e_n^(⊺) B u + e_n^(⊺) K_n e_n + e_n^(⊺) C e_n \
  =>  dot(cal(V)) = 1 / 2 bold(e)_n^(upright(T)) (dot(bold(B)) - 2 bold(C)) bold(e)_n - bold(e)_n^(upright(T)) bold(K)_n bold(e)_n = - bold(e)_n^(upright(T)) bold(K)_n bold(e)_n lt.eq 0 
$


Regarding the utilization of redundancy, a typical choice for $beta$ is
$
   beta = k_(beta) B^(-1) ((partial w(q))/(partial q))
$
where $k_(beta)$ is a signed scalar and $w(q)$ is an additional task function that can be locally optimized. 

= Regulation control


viscous friciotn $tau_f = F dot(q) $. Dynamic model
$
  B(q) dot.double(q) + C(q,dot(q))dot(q) + F dot(q) + g(q) = tau - J^(T)(q) h
$
with $h =0$ and control law of *PD control with gravity compensation*
$
  tau = J^(top) (q) gamma = K_D dot(q) + g(q)
$
we have closed-loop dynamic equation
$
  B dot.double(q)  + C dot(q) + (F + K_D)dot(q) = J^(top) gamma
$
decompose
$
  gamma = vec(
    gamma_p, gamma_o
  )  , quad gamma_p = K_(P p) Delta p_(d e)
$
potential energy
$
  cal(U)_p = 1/2 Delta p_(d e) ^(top) K_(P p) Delta p_(d e), quad 
$
== *Euler angles error*
$
  gamma_o = T^(-T) (phi_e) K_(P o) Delta phi_(d e)
$
$
   cal(U)_o =1/2 Delta phi_(d e)^(top) K_(P o) Delta phi_(d e)
$
Lyapunov function
$
  cal(V) =&  1/2 dot(q)^(top) B dot(q) + cal(U)_p + cal(U)_o\
  =& 1/2 dot(q)^(top) B dot(q) + 1/2 Delta p_(d e) ^(top) K_(P p) Delta p_(d e)+1/2 Delta_(d e)^(top) K_(P o) Delta phi_(d e)
$
$
  (dif )/(dif t) (1/2 dot(q)^(top) B dot(q))=& dot(q)^(top)B dot.double(q) + 1/2 dot(q)^(top) dot(B) dot(q) = dot(q)^(top) [J^(top) gamma - C dot(q) - (F + K_D) dot(q) ] + 1/2 dot(q)^(top) dot(B) dot(q)\
  =& dot(q)^(top) J^(top) gamma - dot(q)^(top)(F + K_D)dot(q) + 1/2 dot(q) ^(top) (dot(B)- 2C)dot(q)\
  =& dot(p)_e^(top) gamma_p + omega_e^(top) gamma_o - dot(q)^(top)(F + K_D)dot(q)
$
$
  dot(cal(U))_p = Delta dot(p)_(d e)^(top) K_(P p) Delta p_(d e) = - dot(p)_e^(top) K_(P p) Delta p_( d e)= - dot(p) _e^(top) gamma_p
$
$
  dot(cal(U))_o = - dot(phi)_e^(top) K_(P p)Delta phi_(d e)= - omega_e^(top) T^(-top) (phi_e) K_(P o) Delta phi_(d e) = -omega_e^(top) gamma_o\
  (omega_e = T(phi_e) dot(phi)_e => phi_e^(top) = omega_e^(top) T^(-top) (phi_e))
$
$
  dot(cal(V))=  - dot(q) ^(top) (F + K_D) dot(q)
$

== *Alternative Euler angles error*

$
  gamma_o = T_e^(-top) (phi_(d e)) K_(P o) phi_(d e), quad cal(U)_o = 1/2 phi_(d e)^(top)K_(P o)phi_(d e)
$
== *Angle/axis error*
$
  gamma_o = K_(P o) o'_(d e), quad cal(U)_o = 2 k_(P o)^(e) epsilon^(top)_(d e) ""^(e)epsilon_(d e)
$

== *Quaternion error*
$
  gamma_o = K_(P o)  R_e ""^(e)epsilon_(d e), quad cal(U)_o = k_(P o)((epsilon_(d e) -1 )^2 + ""^(e) epsilon_(d e)^(top)""^(e) epsilon_(d e)^(top))
$

= Appendix

== The time derivative of $J(q)$

$
  vec(
    dot(p)_e , omega_e
  ) = J(q) dot(q)
$
For revolute joint
$
  J_i = vec(
    z_i times (p_e - p_i),
    z_i 
  ) , quad J= ( J_1, ... , J_6)
$
By $dot(z)_i = omega_i times z_i$
$
  dot(J)_i=  vec(
    (omega_i times z_i) times (p_e - p_i) + z_i times (dot(p)_e - dot(p)_i),
    omega_i times z_i
  ), quad dot(J) = (dot(J)_1, ..., dot(J)_6)
$

== The time derivative of $T(phi_e)$

Consider ZYZ Euler angles
$
  phi_e = vec(
    phi.alt, theta, psi
  ) , quad R_e = R_z (phi.alt) R_y (theta) R_z (psi)
$
Write
$
  omega_e = T(phi_e) dot(phi)_e = e_z dot(phi.alt) + R_z (phi.alt) e_y dot(theta) + R_z (phi.alt) R_y (theta) e_z dot(psi)\
  =>  T  = mat( 0, - sin phi.alt, cos phi.alt sin theta; 0, cos phi.alt, sin phi.alt sin theta; 1, 0, cos theta) 
$
$
   dot(T) = mat( 0, - cos phi.alt dot(phi.alt), - sin phi.alt sin theta dot(phi.alt) + cos phi.alt cos theta dot(theta); 0, - sin phi.alt dot(phi.alt), cos phi.alt sin theta dot(phi.alt) + sin phi.alt cos theta dot(theta); 0, 0, - sin theta dot(theta)) . 
$
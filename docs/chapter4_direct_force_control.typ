Certain tasks require the fulfillment of a precise value of the contact force. Impedance control would be effective only on the assumption that an accurate estimate of the contact stiffness (and geometry of environment) is available.

= Regulation
== Static Model-based Compensation
Recall static model-based compensation in motion regulation control, where we choose
$
  tau = J^(top) gamma - K_D dot(q) + g(q)\
  gamma_p = K_(P p) Delta p_(d e) , quad gamma_o = T^(-top)(phi_e) K_(P p)Delta phi_(d e)
$
It leads to
$
  dot(cal(V)) =- dot(q)^(top)(F + K_D)dot(q)
$
In force regulation, we choose
$
  gamma_p = K_(P p) Delta p_( c e) + f_d , quad gamma_o = T^(-top)(phi_e) K_(P o)Delta phi_(c e) + mu_d\
  gamma = H ^(-top) K_(P) Delta x_(c e) + h_d
$
Then $phi_c, p_c$ can be chosed as *PI control*
$
  p_c = K_(P p) ^(-1) (K_(F p) Delta f + K_(I p) integral_0^(t) Delta f dif sigma.alt)\
  phi_c = K_(P o)^(-1)(K_(F o) Delta mu + K_(I o) integral_0^(t) Delta mu dif sigma.alt )\
  x_c= K^(-1)_(P) (K_F Delta h + K_I integral_0^(t) Delta h dif sigma.alt)
$

== Static Model-based Compensation

Replace static control law $tau = J^(top) gamma - K_D dot(q) + g(q)$ with 
$
   tau = B alpha + C dot(q) + g(q) + J^(top) h
$
and choose
$
  a_p = - K_(D p) dot(p)_e + K_(P p) Delta p_(c e)\
  a_o = T(phi_e) (-K_(D o) dot(phi)_e + K_(P o) Delta phi_(c e)) + dot(T) (phi_e, dot(phi)_e)dot(phi)_e
$
where $p_c, phi_c$ are in form of former PI control. Notice $dot(p)_c,dot(phi)_c,dot.double(p)_c, dot.double(phi)_c$ are not used, and equivalently set to $0$. The reponse of second method is faster since it has lager bandwith of the force loop. 

= Force and Motion Control

parallel control. $p_r= p_c + p_d $ and modeling to environment achieve idea that $p_c$ induces a drawback along constrainted direction while $p_c$ has few contribution along unconstrainted direction.

== Force adn Position Regulation

In this controller, regulation of the end-effector position to the desired components along the plane with regulation of the contact force to the desired component along tHe normal to the plane can be obtained.

In view of parallel control, we choose compliant position
$
  p_r = p_c + p_d 
$
where $p_c$ is obtained from PI control against $Delta f$. Then we use static control law 
$
  gamma_p = K_(P p) (p_r - p_e) + f_d , quad tau = J_p^(top) gamma_p - K_D dot(q) + g
$
We need contact force model of environment
$
  & f= K_f (p_e - p_o) , quad K_f = k_(f ,n)n_c n_c^(top)
$
dynamic system in steady state is
$
  J^(top) (K_(P p) (p_r -p_e) + f_d) = J^(top) f\
  => K_(P p) Delta p_(d e) + (I + K_(F p)) Delta f + K_(I p) integral_0^(oo) Delta f dif sigma.alt =0
$
$f_d$ is taken along $n_c$ in case that end-effector silde along surface. We have 
$
    Delta f = 0 => K_(P p )(p_d - p_e) + K _(I p ) integral_0^(oo) Delta f dif sigma.alt= 0\
    (K_(*) "diagonal") => (I - n_c n_c^(top))(p_d - p_e) =0\
    f_d = k_(f ,n) n_c n_c^(top) (p_e - p_o)=>n_c n_c^(top)  p_e = n_c n_c^(top) p_o + k^(-1)_(f ,n) f_d
$
$
  => p_e = (I - n_c n_c ^(top)) p_d + n_c n_c^(top) (k^(-1)_(f,n) f_d + p_o)\
  f = f_d
$
#figure(
  image("/assets/image-1.png", width: 50%)
)

== Force adn Position Control

For $p_r = p_c + p_d$. We use dynamic model-based control law
$
  tau =  B alpha + C dot(q) + g(q) + J^(top) h\
  a_p = dot.double(p)_d + K_(D p) (dot(p)_d - dot(p)_e) + K_(P p)(p_r p_e), quad quad "in veiw of" dot(p)_c= 0
$
which leads to 
$
  Delta dot.double(p) _(d e) + K_(D p) Delta dot(p)_(d e) + K_(P p) Delta p_(d e) + K_(F p) Delta f + K_(I p) integral_0^(t) Delta f dif sigma.alt = 0
$
Projecting on the axes ${t_(1 c),t_(2c),n_c}$ of $Sigma_c$ and taking $f_(d,t)=0$ yields ($K_(*)$ are diagonal)
$
   Delta dot.double(p) _(d e, t) + K_(D p) Delta dot(p)_(d e, t) + K_(P p) Delta p_(d e, t) =0\
   Delta dot.double(p) _(d e, n) + K_(D p) Delta dot(p)_(d e, n) + K_(P p) Delta p_(d e, n) + K_(F p) Delta f_n + K_(I p) integral_0^(t) Delta f_n dif sigma.alt = 0
$
By model of environment ($f_n = k_(f,n) (p_(e,n) - p_(o,n))$), we have
$
  Delta p_(d e,n) = k_(f,n)^(-1) Delta f_n - d_n\
  d_n = k_(f,n)^(-1) f_(d,n) - p_(d ,n) + p_(o,n)
$
which leads to a third-order system about $Delta f_n$
$ 
  k_(f , n)^(- 1) Delta dot.double(f)_n + k_(f , n)^(- 1) k_(D p) Delta dot(f)_n + (k_(f , n)^(- 1) k_(P p) + k_(F p)) Delta f_n + k_(I p) integral_0^t Delta f_n upright(d) 𝜍 = phi.alt_n \
  phi.alt_n = dot.double(d)_n + k_(D p) dot(d)_n + k_(P p) d_n 
$
The system is stable if 
$
  k_(I p)  < k_(D p) (f_(f,n)^(-1) k_(P p) + k_(F p))
$
$d_n$ is disturbence along normal direction from the mismatch between desired force and position. If $d_n$ is constant, then $phi.alt_n$ is constant and regulation of contact force to the normal component is ensured (thanks to integral of  third-order system). unconstant $d_n$ will increase the force regulation error, but wont affect stability (as long as $phi.alt_(n)$ is bounded).

The choice of the gains for the force control action $K_(F p), K_(I p)$ might depend on the choice of the gains for the position control action ($K_(D p), K_(P p)$) (due to assumption that $dot(p)_c =0$).


A modified parallel control (*full parallel control*) scheme can be devised with the goal of designing the force control action independently of the position control action. Write 
$
   dot(p)_r = dot(p)_c + dot(p)_d\
   dot.double(p)_r = dot.double(p)_c + dot.double(p)_d
$
where $dot.double(p)_c, dot(p)_c$ is solution of 
$
  K_(A p)  dot.double(p)_c + K_(V p) dot(p)_c =Delta f
$
and we choose dynamic model-based control static as
$
  a_p = dot.double(p)_r + K_(D p)(dot(p)_r -dot(p)_e  )+ K_(P p) (p_r -p_e)
$
Projecting the differential equation of $p_c$ on $n_c$ gives
$ k_(A p) dot.double(p)_(c , n) + k_(V p) dot(p)_(c , n) = Delta f_n , $
While assuming that $f_d$ is along $n_c$, it is $p_(c ,t) =0 , dot(p)_(c,t) =0$, which yields
$
  p_(e,t) = p_(d,t)\
  dot(p)_(e,t) = dot(p)_(d,t)
$
While assuming that $p_d$ along $n_c$ is constant, it yields
$ dot(p)_(e , n) = dot(p)_(c , n) \
dot.double(p)_(e , n) = dot.double(p)_(c , n) . \
=>  k_(A p) k_(f , n)^(- 1) dot.double(f)_n + k_(V p) k_(f , n)^(- 1) dot(f)_n = Delta f_n 
$
which implies regulation of the contact force to the desired value along the constrained task direction, for any choice of $k_(A p), k_(V p) > 0$.



== Moment and Orientation control

Choose dynamic model-based control law 
$
  a_o = dot(omega)_r + K_(D o) (omega_r - omega_e) + K_(P o) R_(e) ""^(e) epsilon _(r e)
$
where $""^(e) epsilon_(r e)$is the vector part of the quaternion $Q_(r e) = Q_e ^(-1) * Q_r $. $Q_r omega_r, dot(omega)_r$ come from
$ cal(Q)_r & = cal(Q)_c * cal(Q)_(d c)\
zws^c omega_r & = zws^c omega_c + zws^c omega_(d c)\
zws^c dot(omega)_r & = zws^c dot(omega)_c + zws^c dot(omega)_(d c) $
$Q_(d c), zws^(c) omega_(d c) , zws^(c) dot(omega)_(d c)$ is directly extracted from $R_(d c )$ which is assigned by orientation planer. $Q_c ,zws^(c ) omega_c, zws^(c) dot(omega)_c$ is extracted from $R_c$ which is solution of 
$
  K_(A o) zws^(c)dot(omega)_(c) + K_(V o) zws^(c) omega_c = Delta zws ^(c) mu
$
(Update rule : 
$
   R_c^("next") = R_c e^(hat(zws^(c)omega_c) delta t), quad zws^(c) omega_c^("next") =  zws^(c) omega_c + zws^(c) dot(omega)_c delta t
$)

In term og model of environment
$
  zws^(c)mu = zws^(c)K_(mu) zws^(c)omega_e dif t
$
$zws^(c) mu_n = 0$, a null moment error can be obtained only if the desired moment $zws^(c) mu_d$ has a null component along $n_c$ , i.e. $zws^(c) mu_(d,n) = 0$. Further, a desired rotational motion relative to $Sigma_c$ can be imposed only about $n_c$, i.e. along the unconstrained task direction, and thus the choice $zws^(c) omega_(d c,t ) =0 $  and $zws^(c) dot(omega)_(d c, t)=0$.

error system:
$ k_(A o) zws^c bold(K)_(mu , t)^(- 1 c) dot.double(bold(mu))_t + k_(V o) zws^c bold(K)_(mu , t)^(- 1 c) dot(bold(mu))_t = Delta^c bold(mu)_t $


= Force Tracking 

In order to track force, we replace regulation compliance postion with (Notice that $k_(f,n)$ is assumed to be known)
$
  K_(A p)  dot.double(p)_c + K_(V p) dot(p)_c = phi.alt = k_(f,n)^(-1) f_c = k_(f,n)^(-1) (K_(A p) dot.double(f)_d + K_(V p) dot(f)_d + Delta f)
$
By assuming that $f_d ,dot(f)_d, dot.double(f)_d$ are aligned with$n_c$ and  the component of $p_d$ along $n_c$ is constant, tracking of the desired end-effector position along the unconstrained task directions continues to hold. Moreover, it yields force error system
$
   k_(A p) Delta dot.double(f)_n + K_(V p) Delta dot(f)_n + Delta f_n =0
$
(Project compliance function on $n_c$
$
   K_(A p)  dot.double(p)_(c,n) + K_(V p) dot(p)_(c,n) =  k_(f,n)^(-1) (K_(A p) dot.double(f)_(d,n) + K_(V p) dot(f)_(d,n) + Delta f_(n))
$
Component of $p_(d)$ along $n_c$ is constant
$
  p_(e,n) = p_(r,n) = p_(c,n) + p_(d,n) => dot(p)_(c,n) = dot(p)_(e,n), quad dot.double(p)_(c,n) = dot.double(p)_(e,n)
$
environment contact model
$
  f_n = k_(f,n) (p_(e,n) - p_(o,n)) => dot(f)_n = k_(f,n) dot(p)_(e,n), quad dot.double(f)_n = k_(f,n) dot.double(p)_(e,n)\
  dot(f)_n = k_(f,n) dot(p)_(c,n), quad dot.double(f)_n = k_(f,n) dot.double(p)_(c,n)\
  => k_(A p) Delta dot.double(f)_n + K_(V p) Delta dot(f)_n + Delta f_n =0
$
)

If $k_(f,n)$ is unkown，we have a control scheme to estimate it. Let $hat(epsilon)$ denote the estimate of the inverse fo the contact stiffness. Replace $phi.alt$ in  $K_(A p)  dot.double(p)_c + K_(V p) dot(p)_c = phi.alt$ with
$
  phi.alt = hat(epsilon) f_c + dot(hat(epsilon)) psi 
$
where
$
  dot(psi) + lambda psi = f_c 
$
and update law of $hat(epsilon)$
$
   dot(hat(epsilon)) = gamma psi^(top) Delta f
$
Derivation of error equation. Rewrite
$
  phi.alt = epsilon f_c - tilde(epsilon) f_c- dot(tilde(epsilon)) psi, quad tilde(epsilon) = epsilon - hat(epsilon)\
  => phi.alt = epsilon f_c - dot(zeta) - lambda zeta , quad zeta = tilde(epsilon) psi
$
use desire assumption and environment model, we have
$
   k_(A p) Delta dot.double(f)_n + K_(V p) Delta dot(f)_n + Delta f_n = k_(f,n)  (dot(zeta)_n + lambda zeta_n)
$
it converges when $lambda < k_(V p)/k_(A p)$.







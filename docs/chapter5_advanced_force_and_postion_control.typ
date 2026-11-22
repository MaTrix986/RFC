= Task Space Dynamic
Dynamic model in joint space
$
  B(q) dot.double(q) + C(q,dot(q)) dot(q) + g(q) = tau - J^(top) h 
$
Jaccobian relation
$
  dot(p)_e = J dot(q) , quad dot.double(p)_e =dot(J) dot(q) + J dot.double(q)
$

Dynamic model in task space (position)
$
   B_p (p_e )dot.double(p)_e + C_p (p_e, dot(p)_e) dot(p)_e + g_p (p_e )= u -f\
   B_p = J_p^(-top) B J_p^(-1), quad C_p = J_p^(-top) (C- B J_p^(-1) dot(J)_p) J_p^(-1), quad g_p = J_p^(-top) g
$
$u$ is equivalent control quantity. The joint driving torque can be computed as 
$
  tau = J_p^(top) u
$


= Adaptive Control

== Regulation

*Control law*
$
   u = k_(P p) Delta p_(d e) - k_(D p) dot(p)_e + f_d + k_(F p) Delta f + k_(I p)integral_0^(t) Delta dif sigma.alt + hat(g)_p (p_e)
$

In view of dynamic model in task space, we directly do transformation on control law of *force and postion regulation* 
$
  cases(
    tau = J_p^(top) (k_(P p) (p_r - p_e) + f_d ) - K_D dot(q) + hat(g)(q),
    p_r = p_c + p_d ,
    p_c = K_(P p)^(-1) (K_(F p) Delta f + K_(I p ) integral_0^(t) Delta   f dif sigma.alt )
  ) 
$ 
$
  => u =&  J_p^(-top) tau = K_(P p)(p_r -p_e) + f_d - J_p^(-top) k_D dot(q) + J_p^(-top) g\
  =& k_(P p) Delta p_(d e) - J^(-top)K_D dot(q)  + f_d + k_(F p) Delta f + k_(I p)integral_0^(t) Delta dif sigma.alt + g_p (p_e)
$
*stability analysis* in the  case of $hat(g)_p = g_p$

Let $e = p_(e, oo) - p_e$ denote the deviation of the end-effector position from the equilibrium position. Using equilibrium conclusion of *force and postion regulation* and environment contact model, It can be rewrited as 
$
  cases(
    p_(e, oo) = (I - n_c n_c^(top)) p_d + n_c n_c^(top) (k_(f,n)^(-1) f_d + p_o) ,
    f = k_(f,n) n_c n_c^(top) (p_e - p_o),
    f_d parallel n_c
  ) 
$
$
  => e =& p_(e, oo) -p_e = (I - n_c n_c^(top)) p_d + n_c n_c^(top) (k_(f,n)^(-1) f_d + p_o) -p_e\
  =& (I - n_c n_c^(top)) p_d -p_e + n_c n_c^(top) (k_(f,n)^(-1) Delta f +  n_c n_c^(top) (p_e - p_o) + p_o) \
  =& (I - n_c n_c^(top)) Delta p_(d e) + k_(f,n)^(-1) Delta f
$
decompose $e$
$
  Delta f = k_(f,n) n_c n_c^(top) e 
$
$
  e =& Delta p_(d e) - n_c n_c^(top) (Delta p_(d e) + k_(f,n)^(-1) Delta f) \
  =& Delta p_(d e) - n_c n_c^(top) (p_d -p_e + k_(f,n)^(-1) f_d - n_c n_c^(top) (p_e -p_o)) \
  =& Delta p_(d e) + n_c (k_(f,n)^(-1) f_(d,n) - p_(d,n) + p_(o ,n))\
  =& Delta p_(d e) + d_n n_c 
$
it represents a modified end-effector position error where $d_n$ accounts for the presence of the environment and the desired force along the constrained task direction.

Now we give its *error dynamic model*. 
$
  B_p dot.double(p)_e + C_p dot(p)_e + k_(D p)dot(p)_e = k_(P p)Delta p_(D e) + k'_(F p)Delta f + k_(I p)integral_0^(t) Delta f dif sigma.alt\
  => B_p dot.double(e) + C_p dot(e) + k_(D p)dot(e) + k_(P p)Delta p_(d e) + k'_(F p)Delta f + k_(I p)integral_0^(t) Delta f dif sigma.alt =0 \
  => B_p dot.double(e) + C_p dot(e) + k_(D p)dot(e) + k_(P p)(e - d_n n_c) + k'_(F p)k_(f,n) n_c n_c^(top) e  + k_(I p)integral_0^(t) Delta f dif sigma.alt =0\
  => B_p dot.double(e) + C_p dot(e) + k_(D p)dot(e) + k_(P p) e  + k'_(F p)k_(f,n) n_c n_c^(top) e  + k_(I p) h n_c =0, quad h = n_c^(top) (integral_0^(t) Delta f dif sigma.alt- k_(P p) k_(I p)^(-1) d_n n_c)
$
$
  dot(h) =  k_(f,n) n_c^(top) e 
$
$
  => z = [dot(e)^(top ) quad e^(top) quad h] ^(top), quad dot(z) = A z \
  A = mat(
    -B_p^(-1)(C_p + k_D I), -B_p ^(-1)(k_(P p) I + k'_(F p) k_(f,n) n_c n_c^(top)), -k_(I p ) B_p^(-1) n_c ;
    I, O ,0;
    0^(top), k_(f,n) n_c^(top), 0
  ) 
$
The process of obtaining proper Lyapunov function. Generally, we consider mechanical energy
$
  V_m = 1/2 dot(e) ^(top)  B_p dot(e) + 1/2 k_(P p) e^(top)e + 1/2 k'_(F p) k_(f,n) (n_c^(top) e)^2
$
$
  cases(
    "error dynamic model",
    dot(B)_p = C_p + C_p^(top)
  ) quad => dot(V)_m=- k_(D p) dot(e)^(top) dot(e) - k_(I p ) h n_c^(top) dot(e)
$
To cancel $- k_(I p ) h n_c^(top) dot(e)$, introduce 
$
  V_(e h) = k_(I p)h n_c^(top) e\
  dot(V)_(e h)= k_(I p) k_(f,n) (n_c^(top) e)^2 + k_(I p)h n_c^(top) dot(e)
$
To cancel $k_(I p ) k_(f,n) (n_c^(top) e )^2 $, introduce a negative enough postion error energy
$
   V_(e v)= mu e^(top) B_p dot(e) \
   dot(V)_(e v)= mu dot(e)^(top ) B_p dot(e) + mu e ^(top) C_p  dot(e)  -mu k_(P p) e^(top) e - mu k'_(F p) k_(f,n) (n_c^(top) e)^2 - mu k_(D p) e^(top) dot(e) - mu k_(I p) h n_c ^(top )e
$
To cancel $- mu k_(D p) e^(top) dot(e) ,- mu k_(I p) h n_c ^(top )e$, introduce
$
   V_e= 1/2 mu k_(D p) e^(top) e , quad V_h = mu k_(I p)k^(-1)_(f,n) h^2 \
  dot(V)_e = mu k_(D p) e^(top) dot(e) , quad V_h = mu k_(I p) k_(f.n)^(-1) h dot(h) = mu k_(I p) h n_c^(top) e 
$
Eventually
$
  cal(V) = V_m + V_(e h)  + V_(e v) + V_e + V_h\
  cal(V) = 1/2 z^(top) P z , quad P = mat(
    B_p, mu B_p, 0;
    mu B_p, (k_(P p) + mu k_(D p) )I + k'_(F p)k_(f,n) n_c n_c^(top), k_(I p) n_c;
    0^(top), k_(I p) n_c^(top) , mu k_(I p ) k_(f,n)^(-1)
  ) 
$ 
$
  dot(cal(V)) =&  - k_(D p)dot(e) ^(top) dot(e) + k_(I p)k_(f,n) (n_c^(top) e)^2 + mu dot(e)^(top ) B_p dot(e) + mu e ^(top) C_p  dot(e)  -mu k_(P p) e^(top) e - mu k'_(F p) k_(f,n) (n_c^(top) e)^2\
  =& - dot(e)^(top)(k_(D p) I - mu B_p) dot(e) + mu e^(top) C_p^(top) dot(e) - mu k_(P p) e^(top)e -(mu k'_(F p) - k_( I p))k_(f,n) (n_c^(top) e)^2
$
It is worth remarking that the stiffness coefficient $k_(f,n)$ does not appear in the conditions (5.30), (5.31), which can be easily satisfied since $mu$ is a free parameter not used by the control.

In the case of $hat(g)_p = g_p$. *control law & update law*
$
  u = k_(P p) rho - k_(D p)dot(p)_e + f_d + k_(F p) Delta f + G_p (p_e )hat(pi)_g, quad rho = Delta p_(d e) + k_(P p)^(-1) k_(I p) integral_0^(t )Delta f dif sigma.alt\
  dot(hat(pi))_g = -Upsilon_g^(-1) G_p^(top) (dot(p )- mu rho )
$
$
  (rho = e + k_(P p) ^(-1) k_(I p) h n_c = Delta p_( d e) + k_(P p) ^(-1)k_(I p) integral_0^(t) Delta f dif sigma.alt) 
$
*stability analysis*. We have *error dynamic model*
$
  dot(z) = A z + eta , quad eta = vec(
    B_p^(-1) G_p tilde(pi)_g, 0,0
  ) 
$
Previous Lyapunov function $cal(V)_P = 1/2 z ^(top) P z $ implies
$
   dot(cal(V))_P =&  - dot(e)^(top)(k_(D p) I - mu B_p) dot(e) + mu e^(top) C_p^(top) dot(e) - mu k_(P p) e^(top)e -(mu k'_(F p) - k_( I p))k_(f,n) (n_c^(top) e)^2\
    &+ dot(e)^(top)G_p tilde(pi)_g + mu e ^(top) G_p tilde(pi)_g
$
to cancel $(dot(e)+ mu e)^(top) G_p tilde(pi)_g $, introduce 
$
  V_(pi)  = 1/2 tilde(pi)_g Upsilon_g tilde(pi)_g \
  dot(V)_(pi)=- tilde(pi)_g^(top) Upsilon_g dot(hat(pi))_g
$
with
$
  dot(hat(pi))_g = Upsilon_g^(-1) G_p^(top) (dot(e) + mu e )
$
But there are some problem when using above update law: 1. In view of error dynamic model with gravity compensation error, $h=0$ in not ensured; 2. we cannot compute $e = p_(e,oo)- p_e$. So in order to add a cross term producing $-h^2$, introduce
$
  V_a = rho.alt h n_c^(top)  B_p dot(e) 
$   
$
   dot(V)_a = & rho.alt k_(f , n) n_c^top e n_c^tack.b B_p dot(e) + rho.alt h n_c^tack.b C_p^tack.b dot(e)\
& - rho.alt k_(D p) h n_c^tack.b dot(e)\
 & - rho.alt (k_(P p) + k_(F p)^prime k_(f , n)) h n_c^top e\
 & - rho.alt k_(I p) h^2 + rho.alt h n_c^tack.b G_p tilde(pi)_g 
 $
To cancel $- rho.alt k_(D p) h n_c^tack.b dot(e), - rho.alt (k_(P p) + k_(F p)^prime k_(f , n)) h n_c^top e$, introduce
$
   V_b = rho.alt k_(D p) h n_c^(top) e , quad V_c = 1/2 rho.alt (k_(P p) k_(f.n) ^(-1) + k'_(F p))h^2 \
   dot(V)_b = 𝜚 k_(D p) k_(f , n) (n_c ^top e)^2 + 𝜚 k_(D p) h n_c^tack.b dot(e) , quad dot(V)_c = 𝜚 (k_(P p) + k_(F p)^prime k_(f , n)) h n_c ^top e .
$
new Lyapunov function 
$
  cal(V) = 1/2 z^(top) Q z + 1/2 tilde(pi)_g Upsilon_g tilde(pi)_g
$
$
  Q = P + mat(
  O, O, rho.alt B_p n_c;
  O, O ,rho.alt k_(D p) n_c ;
  rho.alt n_c^(top) B_p, rho.alt k_(D p)n_c^(top) , rho.alt(k_(P p) k_(f,n)^(-1) + k'_(F p))
  ) 
$
$ dot(V) = & - dot(e)^tack.b (k_(D p) I - mu B_p) dot(e) + (mu e^tack.b + 𝜚 h n_c^tack.b) C_p^tack.b dot(e) - mu k_(P p) parallel e parallel^2\
 & - (mu k_(F p)^prime - k_(I p) - 𝜚 k_(D p)) k_(f , n) (n_c^top e)^2 - 𝜚 k_(I p) h^2\
 & + 𝜚 k_(f , n) dot(e)^tack.b B_p n_c n_c^tack.b e\
 & + (dot(e) + mu e + 𝜚 h n_c)^tack.b G_p tilde(pi)_g - tilde(pi)_g^tack.b Upsilon_g dot(hat(pi))_g . $

The last two terms suggest
$
   dot(hat(pi))_g = Upsilon_g^(-1) G_p^(top) (dot(e)+ mu e + rho.alt h n_c )
$
For convience of computation, choose $rho.alt = mu k_(P p)^(-1) k_(I p)$, then 
$
   dot(hat(pi))_g = - Upsilon_g^(-1) G_p^(top) (dot(p)_e - mu rho )
$




== Passivity-Based Control

*Control law & update law*
$
  u =&  hat(B)_p dot(r) + hat(C)_p r + hat(g) + f + lambda_1 (r -dot(p)_e) + lambda_2 rho , quad r = dot(p)_d + k_(P p)rho, \
  =& Y_p (p_e, dot(p)_e,r ,dot(r)  ) hat(pi) + f + lambda_1 (r - dot(p)_e) + lambda_2 rho 
$ 
$
  (rho = e + k_(P p) ^(-1) k_(I p) h n_c = Delta p_( d e) + k_(P p) ^(-1)k_(I p) integral_0^(t) Delta f dif sigma.alt)\
$
$
   dot(hat(pi)) = Upsilon^(-1) Y_p ^(top) sigma , quad sigma = r - dot(p)_e = Delta dot(p)_(d e) + k_(P p) rho 
$

= Output Feedback control


== Regulation



== Passivity-Based Control
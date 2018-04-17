世界坐标系$\large X_gO_gY_g$，小车坐标系$\large X_AO_AY_A$  
坐标为$\large P=[x,y,\theta]^T$，速度为$\large V=[v_x,v_y,w]^T$  
小车在世界坐标系中的坐标$\large P_{InWorld}=[x_g,y_g,\theta_g]^T$  

* <font color=#00ffff size=4>小车在小车坐标系下的速度和在世界坐标系下的速度转换</font>
 

$$\Large 
V_{InWorld}=
\begin{bmatrix}
   v_{xg} \\
   v_{yg} \\
   w_g
\end{bmatrix} 
=
\begin{bmatrix}
   	\cos\theta & -\sin\theta & 0 \\
    \sin\theta & \cos\theta  & 0 \\
    0          &    0        & 1
\end{bmatrix}
\begin{bmatrix}
   v_{xA} \\
   v_{yA} \\
   w_A
\end{bmatrix} 
$$

* <font color=#00ffff size=4>正运动学公式</font>

$$\LARGE \color{#C0Df0f}
V_{InAGV}=
\begin{bmatrix}
   v_{xA} \\
   v_{yA} \\
   w_A
\end{bmatrix}  
=\dfrac r 4
\begin{bmatrix}
   	-1 & 1 & -1&1 \\
    1 & 1  & 1&1 \\
    \dfrac 1 {l_x+l_y}   & -\dfrac 1 {l_x+l_y} &-\dfrac 1 {l_x+l_y}  &\dfrac 1 {l_x+l_y}
\end{bmatrix}
\begin{bmatrix}
   w_1 \\
   w_2 \\
   w_3 \\
   w_4
\end{bmatrix} 
$$
> $l_x$和$l_y$为轮子到$x$轴$y$轴的距离，$w_n$为第$n$个轮子的角速度

* <font color=#00ffff size=4>逆运动学公式</font>

$$\LARGE \color{#C0Df0f}
\begin{bmatrix}
   w_1 \\
   w_2 \\
   w_3 \\
   w_4
\end{bmatrix}= 
\dfrac 1 r
\begin{bmatrix}
   	-1 & 1 & l_x+l_y \\
    1 & 1  & -(l_x+l_y) \\
    -1&1&-(l_x+l_y)\\
    1 & 1 & l_x+l_y
\end{bmatrix}
 \begin{bmatrix}
   v_{xA} \\
   v_{yA} \\
   w_A
\end{bmatrix} 
$$

## 理想情况
* 纯滚动
* 刚体  

设实际当前坐标$\color{#C2CBF2}  P_{Current\_InWorld}=[x_c,y_c,\theta_c]^T$，期望坐标$\color{#C2CBF2} P_{Target\_InWorld}=[x_t,y_t,\theta_t]^T$  
定义误差$\color{#C2CBF2} P_{Error\_InWorld}=P_{Target\_InWorld}-P_{Current\_InWorld}$  

$$ \Large
P_{Error\_InWorld}=[e_x,e_y,e_\theta]^T=
\begin{bmatrix}
   	x_t-x_c \\
    y_t-y_c\\
    \theta_t-\theta_c
\end{bmatrix}
$$

$$ \Large \implies 
P_{Error\_InWorld}'=
\begin{bmatrix}
   	x_t'-x_c' \\
    y_t'-y_c'\\
    \theta_t'-\theta_c'
\end{bmatrix} 
=\begin{bmatrix}
   	v_{xt}-v_{xc} \\
    v_{yt}-v_{yc}\\
    w_t-w_c
\end{bmatrix} 
$$

利用Backstepping方法,选取Lyapunov函数$\large V_1(t)={\dfrac 1 2}{e_x^2}_{(t)}+{\dfrac 1 2}{e_y^2}_{(t)} \implies V_1 \geqslant 0$

$\large \dfrac {dV_1(t)} {dt}=e_x\cdot(v_{xt}-v_{xc})+e_y\cdot(v_{yt}-v_{yc})$

设$\Large v_{xt}-v_{xc}=-c_1 e_x,v_{yt}-v_{yc}=-c_1 e_y$

$\large \implies \dfrac {dV_1(t)} {dt}=-c_1 {e_x^2}-c_2 {e_y^2}\leqslant 0$

选取Lyapunov函数$\large V_2(t)={V_1(t)}+{\sin^2{{\dfrac 1 2}e_\theta}} \implies V_2 \geqslant 0$

$\large \dfrac {dV_2(t)} {dt}={\dfrac {dV_1(t)} {dt}}+ {\dfrac 1 2}{\sin e_\theta \cdot (w_t-w_c)}$


设$\Large w_t-w_c=-c_3 \sin e_\theta$

$\large \implies \dfrac {dV_2(t)} {dt}={\dfrac {dV_1(t)} {dt}}-c_3{\dfrac 1 2}{\sin^2{e_\theta}} \leqslant 0$

>$c_1,c_2,c_3$均为非负常数

### 控制律

>$$ \Large \color{#C0Df0f}
\begin{cases}
   v_{xc}=v_{xt}+c_1e_x  \\
   v_{yc}=v_{xt}+c_2e_y  \\
   w_c=w_t+c_3\sin e_\theta
\end{cases}
$$


## 打滑情况下
定义滑动率$\huge s=\dfrac {wr-v} {wr}$
>$w$为轮子理论角速度，$r$为轮子半径，$v$为轮子实际线速度  

$\huge \implies v=(1-s)wr$  
代入运动学公式

$$\LARGE \color{#C0Df0f}
V_{InAGV}=
\begin{bmatrix}
   v_{xA} \\
   v_{yA} \\
   w_A
\end{bmatrix}  
=\dfrac r 4
\begin{bmatrix}
   	-1 & 1 & -1&1 \\
    1 & 1  & 1&1 \\
    \dfrac 1 {l_x+l_y}   & -\dfrac 1 {l_x+l_y} &-\dfrac 1 {l_x+l_y}  &\dfrac 1 {l_x+l_y}
\end{bmatrix}
\begin{bmatrix}
   (1-s_1)w_1 \\
   (1-s_2)w_2 \\
   (1-s_3)w_3 \\
   (1-s_4)w_4
\end{bmatrix} 
$$
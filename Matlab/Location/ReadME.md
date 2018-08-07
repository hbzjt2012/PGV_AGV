# 使用卡尔曼滤波对AGV进行定位

## 融合陀螺仪测得角度$\theta,\omega$与编码器计算得到的$\omega$

状态量$x_t$是t时刻状态的真实值

$$ \Large
  x_t=\begin{bmatrix}
   \hat{\theta_t} \\
   \hat{\omega_t}
\end{bmatrix}
$$

状态量$\mu_t$是t时刻状态的估计值

$$ \Large
  \mu_t=\begin{bmatrix}
   \theta_t \\
   \omega_t
\end{bmatrix}
$$

输入量$u_t$是t时刻由编码器计算得到的角速度

$$\Large u_t=\omega_t$$

测量量$z_t$是t时刻由陀螺仪测量得到的角度、角速度

$$ \Large
  z_t=\begin{bmatrix}
   \theta_z \\
   \omega_z
\end{bmatrix}
$$

1. 预测状态量$\Large \overline{\mu}$  
  $\Large \overline{\mu}=A\mu_{t-1}+Bu_t$
2. 预测协方差矩阵$\Large \overline{\Sigma_t}$  
  $\Large \overline{\Sigma_t}=A\Sigma_{t-1}A^T+R$
  > $R$为预测噪声协方差矩阵
3. 计算卡尔曼增益矩阵$\large K_t$  
  $\large K_t=\overline{\Sigma_t}C^T/(C\overline{\Sigma_t}C^T+Q)$
  > $Q$为测量噪声协方差矩阵
4. 更新状态量$\Large \mu_t$  
$\Large \mu_t=\overline{\mu}+K_t(z_t-C\overline{\mu})$
5. 更新协方差矩阵$\Large \Sigma_t$  
$\Large \Sigma_t=(I-K_tC)\overline{\Sigma_t}$  
  > $I$是单位矩阵

其中

$$
A=\begin{bmatrix}
   1 &0 \\
   0 &0
\end{bmatrix}
,
B=\begin{bmatrix}
   \Delta t \\
   1
\end{bmatrix}
,
C=\begin{bmatrix}
   1 &0 \\
   0 &1
\end{bmatrix}
$$

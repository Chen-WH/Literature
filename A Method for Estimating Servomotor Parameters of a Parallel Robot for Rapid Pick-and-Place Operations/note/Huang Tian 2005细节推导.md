# Huang Tian 2005细节推导

## Dynamic Formulation

### Inverse Kinematics

$$
(\pm e+l_1\cos\theta_{1i}-x)^2+(l_1\sin\theta_{1i}-y)^2=l_2^2\\
2(\pm e-x)l_1\cos\theta_{1i}-2yl_1\sin\theta_{1i}=l_2^2-e^2-l_1^2-x^2-y^2\\
\Longrightarrow A\cos\theta_{1i}+B\sin\theta_{1i}=C\\
万能公式\quad\theta_{1i}=2\arctan(\frac{B\pm\sqrt{B^2+A^2-C^2}}{A+C})\\
\boldsymbol{w}_i=\left(\boldsymbol{r}-\pm e\boldsymbol{e}_1-l_1\boldsymbol{u}_i\right)/l_2\\
$$

### Jacobian

$$
\boldsymbol{r}-e\boldsymbol{e}_1-l_1\boldsymbol{u}_1=l_2\boldsymbol{w}_1\\
\boldsymbol{v}-l_1\dot{\theta}_1\left(\boldsymbol{k}\times\boldsymbol{u}_1\right)=l_2\dot{\theta}_2\left(\boldsymbol{k}\times\boldsymbol{w}_1\right)\\
\boldsymbol{v}\cdot\boldsymbol{w}_1-l_1\dot{\theta}_1\left(\boldsymbol{k}\times\boldsymbol{u}_1\right)\cdot\boldsymbol{w}_1=l_2\dot{\theta}_2\left(\boldsymbol{k}\times\boldsymbol{w}_1\right)\cdot\boldsymbol{w}_1=0\\
\dot{\theta}_1=\frac{\boldsymbol{v}\cdot\boldsymbol{w}_1}{l_1\boldsymbol{k}\cdot\left(\boldsymbol{u}_1\times\boldsymbol{w}_1\right)}=\frac{\boldsymbol{v}\cdot\boldsymbol{w}_1}{l_1\boldsymbol{u}_1^T\boldsymbol{H}\boldsymbol{w}_1}\quad\boldsymbol{H}=[0, 1; -1, 0]\\
\dot{\theta}_1=\frac{\boldsymbol{w}_1^T}{l_1\boldsymbol{u}_1^T\boldsymbol{H}\boldsymbol{w}_1}\boldsymbol{v}=\boldsymbol{J}_1\boldsymbol{v}\\
$$

核心步骤：消去角速度项
$$
\boldsymbol{v}-l_1\dot{\theta}_1\left(\boldsymbol{k}\times\boldsymbol{u}_1\right)=l_2\dot{\theta}_2\left(\boldsymbol{k}\times\boldsymbol{w}_1\right)\\
\boldsymbol{a}=l_1\ddot{\theta}_1\left(\boldsymbol{k}\times\boldsymbol{u}_1\right)+l_2\ddot{\theta}_2\left(\boldsymbol{k}\times\boldsymbol{w}_1\right)+l_1\dot{\theta}_1^2\boldsymbol{k}\times\left(\boldsymbol{k}\times\boldsymbol{u}_1\right)+l_2\dot{\theta}_2^2\boldsymbol{k}\times\left(\boldsymbol{k}\times\boldsymbol{w}_1\right)\\
\boldsymbol{a}=l_1\ddot{\theta}_1\left(\boldsymbol{k}\times\boldsymbol{u}_1\right)+l_2\ddot{\theta}_2\left(\boldsymbol{k}\times\boldsymbol{w}_1\right)+l_1\dot{\theta}_1^2\left(-\boldsymbol{u}_1\right)+l_2\dot{\theta}_2^2\left(-\boldsymbol{w}_1\right)\\
\boldsymbol{a}\cdot\boldsymbol{w}_1=l_1\ddot{\theta}_1\boldsymbol{k}\cdot\left(\boldsymbol{u}_1\times\boldsymbol{w}_1\right)-l_1\dot{\theta}_1^2\left(\boldsymbol{u}_1\cdot\boldsymbol{w}_1\right)-l_2\dot{\theta}_2^2\\
又有\quad\boldsymbol{v}\cdot\boldsymbol{w}_1=l_1\dot{\theta}_1\boldsymbol{k}\cdot\left(\boldsymbol{u}_1\times\boldsymbol{w}_1\right)\quad\boldsymbol{v}\cdot\boldsymbol{u}_1=-l_2\dot{\theta}_2\boldsymbol{k}\cdot\left(\boldsymbol{u}_1\times\boldsymbol{w}_1\right)\\
\ddot{\theta}_1=\frac{\boldsymbol{a}\cdot\boldsymbol{w}_1+l_1\dot{\theta}_1^2\left(\boldsymbol{u}_1\cdot\boldsymbol{w}_1\right)+l_2\dot{\theta}_2^2}{l_1\boldsymbol{u}_1^T\boldsymbol{H}\boldsymbol{w}_1}=\boldsymbol{Ja}+\frac{(\boldsymbol{v}\cdot\boldsymbol{w}_1)^2\left(\boldsymbol{u}_1\cdot\boldsymbol{w}_1\right)/l_1+(\boldsymbol{v}\cdot\boldsymbol{u}_1)^2/l_2}{l_1\left(\boldsymbol{u}_1^T\boldsymbol{H}\boldsymbol{w}_1\right)^3}\\
\ddot{\theta}_1=\boldsymbol{Ja}+\frac{(\boldsymbol{v}\cdot\boldsymbol{w}_1)^2\left(\boldsymbol{u}_1\cdot\boldsymbol{w}_1\right)/l_1+(\boldsymbol{v}\cdot\boldsymbol{u}_1)^2/l_2}{l_1\left(\boldsymbol{u}_1^T\boldsymbol{H}\boldsymbol{w}_1\right)^3}\\
$$

### Inverse Dynamics

$$
(-m\boldsymbol{a}-mg\boldsymbol{e}_2)^T\delta\boldsymbol{r}+(\boldsymbol{\tau}-I_A\ddot{\theta}_1-\boldsymbol{\tau}_{Ag})^T\delta\boldsymbol{\theta}_1=0\\
\boldsymbol{\tau}=(I_A+m\boldsymbol{J}^{-T}\boldsymbol{J}^{-1})\ddot{\theta}_1-m\boldsymbol{J}^{-T}\boldsymbol{J}^{-1}f(\dot{\theta}_1)+\boldsymbol{\tau}_{Ag}+mg\boldsymbol{J}^{-T}\boldsymbol{e}_2\\
\boldsymbol{\tau}=(I_A\boldsymbol{J}+m\boldsymbol{J}^{-T})\boldsymbol{a}+I_Af(\boldsymbol{v})+\boldsymbol{\tau}_{Ag}+mg\boldsymbol{J}^{-T}\boldsymbol{e}_2,\quad \boldsymbol{G}=I_A\boldsymbol{J}+m\boldsymbol{J}^{-T}\\
P_i=\tau_i\dot{\theta}_{1i}\\
$$

这里原文有一处勘误 $\eta=m/I_A$

## Servomotor Parameter Estimation

### Maximum Angular Velocity

$$
j=\frac{\pi N_{\max }}{30 \dot{\Theta}_{\max }}\quad
\dot{\Theta}=\max\left(\sqrt{\boldsymbol{J_1J_1}^T}\right)\boldsymbol{v}_{\max}
$$

### Moment of Inertia

$$
I_{L} /\left(3 j^{2}\right)<I_{M}<I_{L} / j^{2}\quad
I_L=I_A+m\boldsymbol{J}^{-T}\boldsymbol{J}^{-1}
$$

### Maximum torque

$$
\tau_{M \max }=T_{\max } / j\quad
T_{\max }=\max\left(\sqrt{\boldsymbol{G_1G_1}^T}\right)\boldsymbol{a}_{\max}
$$


# Kalman Filter

## Dynamics


$$
\newcommand{\x}{\mathbf{x}}
\newcommand{\T}{\Gamma}
\newcommand{\q}{\mathbf{q}}
\newcommand{\v}{\mathbf{v}}
\newcommand{\ba}{\mathbf{b}_{a}}
\newcommand{\bw}{\mathbf{b}_{\omega}}
\newcommand{\gps}{\mathbf{c}_{\mathrm{gps}}}
\newcommand{\gal}{\mathbf{c}_{\mathrm{gal}}}
\newcommand{\glo}{\mathbf{c}_{\mathrm{glo}}}
\newcommand{\pb}{\mathbf{p}_{g/b}^b}
\newcommand{\p}{\mathbf{p}}
\newcommand{\g}{\mathbf{g}}
\newcommand{\a}{\mathbf{a}}
\newcommand{\z}{\mathbf{z}}
\newcommand{\u}{\mathbf{u}}
\newcommand{\Te}{\Gamma_I^e}
\newcommand{\bs}{\boldsymbol{\beta}_s}
\newcommand{\w}{\boldsymbol{\omega}}
\newcommand{\ew}{\boldsymbol{\eta}_\omega}
\newcommand{\ea}{\boldsymbol{\eta}_a}
\newcommand{\skew}[1]{\lfloor#1\rfloor_\times}
$$


Remember that the the dual quaternion is defined as
$$
\T_I^b = \q_I^b + \frac{1}{2}  \begin{pmatrix} 0 \\\\ \p_{b/a}^b \end{pmatrix} \cdot \q_I^b
$$
and has dynamics
$$
\dot{\Gamma}_I^b = \frac{1}{2}\Gamma_I^b \cdot \left( \begin{pmatrix} 0 \\\\ \w \end{pmatrix} + \epsilon \begin{pmatrix} 0 \\\\ \v \end{pmatrix}\right)
$$

The state vector is defined as
$$
\mathbf{x} =
\begin{pmatrix}
\T_I^b \\
\v_{b/I}^b \\
\ba \\
\bw \\
\gps \\
\gal \\
\glo \\
\pb \\
\Te \\
\bs
\end{pmatrix}
$$

and the dynamics are given as

$$
\begin{align}
\dot{\T_I^b} &= \begin{pmatrix} \w_{b/I}^b \\ \v \end{pmatrix} \\
\dot{\v}_{b/I}^b &= R_I^b \g^I + \a_{b/I}^b\\
\dot{\ba} &= 0 \\
\dot{\bw} &= 0 \\
\dot{\gps} &= 0 \\
\dot{\gal} &= 0 \\
\dot{\glo} &= 0 \\
\dot{\pb} &= 0 \\
\dot{\Te} &= 0 \\
\dot{\bs} &= 0
\end{align}
$$

## Error-state dynamics

The error-state of our system is defined as

$$
\begin{align}
\tilde{\x} &= \x - \hat{\x} \\
\tilde{\u} &= \u - \hat{\u}
\end{align}
$$

and we use the left-difference in the case of group state members, such as

$$
\tilde{\T} = \log\left(\T^{-1} \cdot \hat{\T}\right)
$$

and if we attach frames to it, (and remember that dual-quaternions concatenate backwards)

$$
\begin{align}
\tilde{\T} &= \log\left(\left(\T_I^b\right)^{-1} \cdot \hat{\T_I^\hat{b}}\right) \\
 &= \log\left(\T_b^I \cdot \hat{\T_I^\hat{b}}\right) \\
 &= \log\left(\T_b^\hat{b}\right) \\
 &= \xi_b^\hat{b} \\
 &= \begin{pmatrix} \tilde{r}_b^\hat{b} \\ \tilde{p}_b^\hat{b} \end{pmatrix}
\end{align}
$$


The true angular velocity is modeled as

$$\w = \bar{\w} - \bw + \ew$$

which means that our estimated angular velocity input is

$$\hat{\w} = \bar{\w} - \hat{\bw}$$

and the error-state inputs is

$$
\begin{align}
\tilde{\w} &= \w - \hat{\w} \\
           &= \tilde{\w} - \bw + \ew - \left(\bar{\w} - \hat{\bw}\right) \\
           &= -\tilde{\bw} + \ew
\end{align}
$$

and in likewise manner

$$
\tilde{\a} = -\tilde{\ba} + \ea.
$$


We can now derive our error-state dynamics

$$
\dot{\tilde{\x}} = \frac{d}{dt}\left(\x - \hat{\x}\right)
$$

$$
\begin{align}
\dot{\tilde{\T_I^b}} &= \begin{pmatrix}
    \w_{b/I}^b - \w_{\hat{b}/I}^\hat{b} \\
    \v_{b/I}^b - \v_{\hat{b}/I}^\hat{b} \\
\end{pmatrix} \\
\dot{\tilde{\v}} &= \v_{b/I}^b - \hat{₂v}_{\hat{b}/I}^\hat{b}
\end{align}
$$

and the constants are all zero.


Taking each term one, by one, we can use the approximation $R\left(\exp_\q \theta\right) \approx I - \skew{\theta}$

$$
\newcommand{\what}{\hat{\w}_{\hat{b}/I}^\hat{b}}
\newcommand{\wnormal}{\w_{b/I}^b}
\newcommand{\wtilde}{\tilde{\w}_{\hat{b}/b}^\hat{b}}
\newcommand{\rtilde}{\tilde{r}_b^\hat{b}}
\begin{align}
\dot{\tilde{r}}_b^\hat{b} &= \wnormal - \what \\
    &= R_b^\hat{b}\wnormal - \what \\
    &\approx \left(I - \skew{\rtilde}\right)^\top \wnormal - \what \\
    &= \left(I - \skew{\rtilde}\right)^\top \left(\what + \wtilde\right) - \what \\
    &= \left(\what + \wtilde\right) - \skew{\rtilde}^\top \left(\what + \wtilde\right) - \what \\
    &= \wtilde - \skew{\rtilde}^\top \left(\what + \wtilde\right) \\
    &\approx \wtilde - \skew{\rtilde}^\top \what \\
    &= \left(-\tilde{\bw} + \ew\right) + \skew{\rtilde} \left(\bar{\w} - \hat{\bw}\right) \\
    &= -\skew{\bar{\w} - \hat{\bw}} \rtilde - \tilde{\bw} + \ew
\end{align}
$$

$$
\begin{align}
\newcommand{\vhat}{\hat{\v}_{\hat{b}/I}^\hat{b}}
\newcommand{\vnormal}{\v_{b/I}^b}
\newcommand{\vtilde}{\tilde{\v}_{\hat{b}/b}^\hat{b}}
\newcommand{\rtilde}{\tilde{r}_b^\hat{b}}
\dot{\tilde{p}}_b^\hat{b} &= \vnormal - \vhat \\
    &= R_b^\hat{b}\vnormal - \vhat \\
    &\approx \left(I - \skew{\rtilde}\right)^\top \vnormal - \vhat \\
    &= \left(I - \skew{\rtilde}\right)^\top \left(\vhat + \vtilde\right) - \vhat \\
    &= \left(\vhat + \vtilde\right) - \skew{\rtilde}^\top \left(\vhat + \vtilde\right) - \vhat \\
    &= \vtilde - \skew{\rtilde}^\top \left(\vhat + \vtilde\right) \\
    &\approx \vtilde - \skew{\rtilde}^\top \vhat \\
    &= \vtilde + \skew{\rtilde} \vhat \\
    &= \vtilde -\skew{\vhat} \rtilde
\end{align}
$$

$$
\begin{align}
\newcommand{\anormal}{\a_{b/I}^b}
\newcommand{\ahat}{\hat{\a}_{\hat{b}/I}^\hat{b}}
\newcommand{\atilde}{\tilde{\a}_{\hat{b}/b}^\hat{b}}
\dot{\tilde{\v}} &= \v_{b/I}^b - \hat{\v}_{\hat{b}/I}^\hat{b} \\
    &= \left(R_I^b \g^I + \anormal \right) - \left(R_I^\hat{b} \g^I + \ahat\right) \\
    &= R_b^\hat{b}\left(R_I^b \g^I + \anormal \right) - \left(R_I^\hat{b} \g^I + \ahat\right) \\
    &= R_I^\hat{b}\g^I + R_b^\hat{b}\anormal - \left(R_I^\hat{b} \g^I + \ahat\right) \\
    &= R_b^\hat{b}\anormal - \ahat \\
    &= R_b^\hat{b}\left(\ahat + \atilde\right) - \ahat \\
    &\approx \left(I - \skew{\rtilde}\right)^\top\left(\ahat + \atilde\right) - \ahat \\
    &= \left(\ahat + \atilde\right) - \skew{\rtilde}^\top\left(\ahat + \atilde\right) - \ahat \\
    &= \atilde + \skew{\rtilde}\left(\ahat + \atilde\right) \\
    &\approx \atilde + \skew{\rtilde}\ahat  \\
    &= \atilde + \skew{\rtilde}\left(\bar{\a} - \hat{\ba}\right)  \\
    &= \left(-\tilde{\ba} + \ea\right) - \skew{\bar{\a} - \hat{\ba}}\rtilde
\end{align}
$$

## Measurement Models

The first and simplest measurement model is the position and velocity measurement from a point-positioning solution

$$
\z = \begin{pmatrix}\p_{g/e}^e \\ \v_{g/e}^e\end{pmatrix}
$$

The measurement model is then
$$
\begin{align}
\z &= h\left(\x\right) \\\\
    &=\begin{pmatrix}
        R_I^e \left(R_b^I\left(\p_{g/b}^b  + \p_{b/I}^b\right)\right) + \p_{I/e}^e\\\\
        R_I^e R_b^I\left(\v_{g/b}^b + \v_{b/I}^b\right) + \cancelto{0}{\v_{I/e}^e}
    \end{pmatrix}\\\\
    &=\begin{pmatrix}
        \T_I^e \otimes \left( \left(\T_I^b\right)^{-1} \otimes \left( \p_{g/b}^b \right)\right)\\\\
        R_I^e R_b^I\left(\w_{b/I}^b \skew{\p_{g/b}^b} + \v_{b/I}^b\right)
    \end{pmatrix}
\end{align}
$$

which means that the right-jacobians are

$$
\frac{\partial h}{\partial \T_I^b} =
\begin{pmatrix}
    R_I^e R_b^I \skew{p_{g/b}^b}  & -R_I^e R_b^I \\
   R_I^e \left(R_b^I\skew{\v_{g/b}^b + \v_{b/I}^b}\right) & 0
\end{pmatrix}
$$

$$
\frac{\partial h}{\partial \v_{b/I}^b} =
\begin{pmatrix}
    0 \\
   R_I^e R_b^I
\end{pmatrix}
$$

$$
\frac{\partial h}{\partial \p_{g/b}^b} =
\begin{pmatrix}
    R_I^e R_b^I \\
    0
\end{pmatrix}
$$

$$
\frac{\partial h}{\partial \T_I^e} =
\begin{pmatrix}
    -\skew{ \p_{g/e}^e} & I\\
    R_I^e & 0
\end{pmatrix}
$$

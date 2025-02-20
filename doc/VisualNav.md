# Visual Navigation

The equation that describes the projection from 3-D coordinate space to 2-D image space is given below:

$$
\alpha \mathbf{x} = \mathbf{K} \bigl[ \mathbf{R} | \mathbf{T} \bigr] \mathbf{X}
$$

where the $\alpha$ is a constant term that ensures the third element in $\mathbf{x}$ is equal to 1, making the first two elements in $\mathbf{x}$ equal to the $u$ and $v$ pixel locations within the image. So the left hand of the equation looks like:

$$
\alpha
\begin{bmatrix}
u \\
v \\
1
\end{bmatrix}
$$

The [3x3] upper-triangular matrix $\mathbf{K}$ describes the camera intrinsics. This matrix contains the focal length $(f_x, f_y)$ and principal point $(p_x, p_y)$ of the imaging system:

$$
\mathbf{K} = 
\begin{bmatrix}
f_x & 0   & p_x \\
0   & f_y & p_y \\
0   & 0   & 1   \\
\end{bmatrix}
$$

The [3x4] matrix $\bigl[ \mathbf{R} | \mathbf{T} \bigr]$ consists of a rotation matrix $\mathbf{R}$, with a translation matrix $\mathbf{T}$ appended on the final column. Together, these define the extrinsic coordinates of the camera origin in the world coordinate frame. 
The [4x1] homogeneous vector $\mathbf{X}$ is the location of the observed object in world coordinates, with a 1 appended to the bottom row.
Expanded, the complete equation looks as follows:

$$
\begin{equation}
\tag{1}
\alpha
\begin{bmatrix}
u \\
v \\
1
\end{bmatrix}
=
\begin{bmatrix}
f_x & 0   & p_x \\
0   & f_y & p_y \\
0   & 0   & 1   \\
\end{bmatrix}
\begin{bmatrix}
R_1 & R_2  & R_3 & T_x\\
R_4 & R_5  & R_6 & T_y\\
R_7 & R_8  & R_9 & T_z\\
\end{bmatrix}
\begin{bmatrix}
X \\
Y \\
Z \\
1
\end{bmatrix}
\end{equation}
$$

We can separate $\mathbf{T}$ from this equation, and isolate it to give:
$$
\mathbf{T} =  \mathbf{K}^{-1} ( \alpha \mathbf{x} - \mathbf{K} \mathbf{R} \mathbf{X})
$$
where we have dropped the 1 from bottom row of the vector $\mathbf{X}$ to give it dimension [3x1]. The matrices $\mathbf{T}$, $\mathbf{x}$ and $\mathbf{R}$ are implicitly functions of time. The intrinsics matrix $\mathbf{K}$ is fixed, thus is time invariant. We also assume that the world points which we are observing are fixed and not moving in the scene, so the vector $\mathbf{X}$ is also time invariant. To be more explicit about this, we can expand, simplify and write it as:
$$
\mathbf{T}(t) = \alpha\ \mathbf{K}^{-1} \mathbf{x}(t) - \mathbf{R}(t) \mathbf{X}
$$


 Taking the time derivative of both sides, we get:
$$
\begin{equation}
\tag{2}
\mathbf{\dot{T}} = \alpha \mathbf{K}^{-1} \mathbf{\dot{x}} - \mathbf{\dot{R}} \mathbf{X}
\end{equation}
$$

The time derivative of a rotation matrix is given by 

$$
\mathbf{\dot{R}} = 
[\bm{\omega}]_{\times} \mathbf{R}(t)\ \ \ ; \ \ \ 

[\bm{\omega}]_{\times} = 
\begin{bmatrix}
0         & -\omega_z & \omega_y  \\
\omega_z  & 0         & -\omega_x \\
-\omega_y & \omega_x  & 0
\end{bmatrix}
$$
where $[\bm{\omega}]_{\times}$ is the skew-symmetric rate matrix, as measured by the gyroscope. Note that the gyroscope measures the rate of the aircraft, so this measurement must first be converted from the aircraft frame to the camera frame (typically by a time-invariant rotation matrix). 

Our interest is in solving equation (2) to estimate the rate of translation vector $\mathbf{\dot{T}}$. Lucky for us, many of these variables are already known. 
It is possible to infer the location of an observed point $\mathbf{X}$. 
We assume that the altitude of the aircraft above the ground is known (either by barometric altitude & terrain mapping, or by LiDAR), thus $T_z$ is known. In aerial applications, we assume that the observed tracking point is on the ground, and therefore $Z$ is equal to 0. The rotation $\mathbf{R}$ of the camera system is estimated from the onboard AHRS, thus all elements $R_i$ are known. The time derivative of $\mathbf{x}$ can be observed across sequential frames, simply given by $\dot{u} = (u_{\tau} - u_{t}) / (\tau - t)$ and $\dot{v} = (v_{\tau} - v_{t}) / (\tau - t)$. It can be seen in equation (1) that the scalar $\alpha$ is equal to the bottom row of the equation, for which all variables are known. The intrinsic matrix $\mathbf{K}$ is computed during the calibration process. 
Therefore the instantaneous velocity of the camera $\mathbf{\dot{T}}$ can be estimated from the translational velocity of a pixel across the image plane.
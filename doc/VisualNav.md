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

If we set \mathbf{T} to 0, such that the coordinate system is centred on the aircraft, we can simplify to give:
$$
\alpha \mathbf{x} = \mathbf{K}\mathbf{R}\mathbf{X}
$$
where we have dropped the 1 from bottom row of the vector $\mathbf{X}$ to give it dimension [3x1]. 
The vector $\mathbf{x}$ and matrix $\mathbf{R}$ are implicitly functions of time. The intrinsics matrix $\mathbf{K}$ is fixed, thus is time invariant. We also assume that the aircraft is fixed in space, and the world is moving beneath it. In reality the opposite is true, but we can account for this simply by inverting the sign of the result. We can find the point $\mathbf{X}$ by multiplying by $\mathbf{KR}^{-1}$ on both sides
$$
\mathbf{X} = \alpha \mathbf{(KR)}^{-1} \mathbf{x}
$$
The scale factor $\alpha$ can be computed with knowledge of the aircraft's altitude above ground level. We find alpha such that the third element of $\alpha \mathbf{KR}^{-1}\mathbf{x}$ is equal to the altitude AGL. Therefore, the ground displacement can be measured directly, by computing $\mathbf{X}$ at two different points in time from the same tracking point. 
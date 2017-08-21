###目录    
- [1 预积分推导](#1)
  - [1.1 离散状态下预积分方程](#1.1)
  - [1.2 离散状态下误差状态方程](#1.2)

####参考文献
[1] [VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen (techincal report)](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/support_files/paper/tro_technical_report.pdf)    
<span id="[2]">[2] [Solà J. Quaternion kinematics for the error-state KF[M]// Surface and Interface Analysis. 2015.](http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf)</span>   
<span id="[3]">[3] [Solà J. Yang Z, Shen S. Monocular Visual–Inertial State Estimation With Online Initialization and Camera–IMU Extrinsic Calibration[J]. IEEE Transactions on Automation Science & Engineering, 2017, 14(1):39-51.](http://ieeexplore.ieee.org/document/7463059/?reload=true&arnumber=7463059)</span>     
<span id="[3]">[4] [Engel J, Schöps T, Cremers D. LSD-SLAM: Large-scale direct monocular SLAM[C]//European Conference on Computer Vision. Springer, Cham, 2014:834-849.](http://xueshu.baidu.com/s?wd=paperuri%3A%28b5b76005594ab735af07c8a12035cc20%29&filter=sc_long_sign&tn=SE_xueshusource_2kduw22v&sc_vurl=http%3A%2F%2Flink.springer.com%2F10.1007%2F978-3-319-10605-2_54&ie=utf-8&sc_us=18106927782980728032)</span>    
------
###<span id="前言">前言      
&emsp;&emsp;Vins-Mono是视觉与IMU的融合中的经典之作，其定位精度可以媲美OKVIS，而且具有比OKVIS更加完善和鲁棒的初始化以及闭环检测过程。同时VINS-Mono也为该邻域树立了一个信标吧，视觉SLAM的研究和应用会更新偏向于 __单目+IMU__。因为在机器人的导航中，尤其是无人机的自主导航中，单目不具有RGBD相机(易受光照影响、获取的深度信息有限)以及双目相机(占用较大的空间、。。。)的限制，能够适应室内、室外及不同光照的环境，具有较好的适应性。而且在增强和虚拟现实中，更多的移动设备仅具有单个相机，所以单目+IMU也更符合实际情况。      
&emsp;&emsp;那么为什么要进行视觉与IMU的融合呢，自己总结的主要有以下几点：     
- 视觉与IMU的融合可以借助IMU较高的采样频率，进而提高系统的输出频率。
- 视觉与IMU的融合可以提高视觉的鲁棒性，如视觉SLAM因为某些运动或场景出现的错误结果。
- 视觉与IMU的融合可以有效的消除IMU的积分漂移。
- 视觉与IMU的融合能够校正IMU的Bias。
- 单目与IMU的融合可以有效解决单目尺度不可观测的问题。

&emsp;&emsp;上面总结了视觉与IMU融合的几个优点，以及与单目融合可以解决单目相机尺度不可测的问题。但是单目相机的尺度不可测也是具有优点的，参考[[4]](#[4])。单目尺度不确定的优点主要有两方面：单目尺度的不确定性，可以对不同的规模大小的环境空间之间进行游走切换，祈祷无缝连接的作用，比如从室内桌面上的环境和大规模的户外场景；诸如深度或立体视觉相机，这些具备深度信息的传感器，它们所提供的可靠深度信息范围，是有限制的，故而不能像单目相机那样具有尺度灵活性的特点。
<h3 id="1">1 预积分的推导 </h3>
<h4 id="1.1">__1.1 离散状态下预积分方程__</h4>
关于这部分的论文和代码中的推导，可以参考文献[[2]](#[2])中Appendx部分“A Runge-Kutta numerical integration methods”中的欧拉法和中值法。
$$
w_{k}^{{}'}=\frac{w_{k+1}+w_{k}}{2}-b_{w}
\tag{1.1}
$$
$$
q _{i+1}=q _{i}\otimes \begin{bmatrix}
1
\\
0.5w_{k}^{{}'}
\end{bmatrix}
\tag{1.2}
$$
$$
a_{k}^{{}'}=\frac{q_{k}(a_{k}+n_{a0}-b_{a_{k}})+q_{k+1}(a_{k+1}++n_{a1}-b_{a_{k}})}{2}
\tag{1.3}
$$
$$
\alpha _{i+1}=\delta\alpha _{i}+\beta _{i}t+0.5a_{k}^{{}'}\delta t^{2}
\tag{1.4}
$$
$$
\beta _{i+1}=\delta\beta _{i}+a_{k}^{{}'}\delta t
\tag{1.5}
$$

<h4 id="1.1">__1.2 离散状态下误差状态方程__</h4>     &emsp;&emsp;论文中Ⅱ.B部分的误差状态方程是连续时间域内，在实际代码中需要的是离散时间下的方程式，而且在前面的预积分方程中使用了中值法积分方式。所以在实际代码中和论文是不一致的。在推导误差状态方程式的最重要的部分是对 $\delta \theta _{k+1}$ 部分的推导。    
&emsp;&emsp;由泰勒公式可得：
$$
\delta \theta _{k+1} = \delta \theta _{k}+\dot{\delta \theta _{k}}\delta t
\tag{2.1}
$$
依据参考文献[[2]](#[2])中 "5.3.3 The error-state kinematics"中公式(222c)及其推导过程有：
$$
\dot{\delta \theta _{k}}=-[w_{m}-w_{b}]_{\times }\delta \theta _{k}-\delta w_{b}-w_{n}
$$
对于中值法积分下的误差状态方程为：
$$
\dot{\delta \theta _{k}}=-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta \theta _{k}-\delta b_{g_{k}}+\frac{n_{w0}+n_{w1}}{2}
\tag{2.2}
$$
将式(2.2)带入式(2.1)可得：
$$
\delta \theta _{k+1} =(I-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta t) \delta \theta _{k} -\delta b_{g_{k}}\delta t+\frac{n_{w0}+n_{w1}}{2}\delta t
\tag{2.3}
$$
这部分也可以参考，文献[[2]](#[2])中“7.2 System kinematics in discrete time”小节。    
&emsp;&emsp;接下来先推导 $\delta \beta _{k+1}$ 部分，再推导 $\delta \alpha _{k+1}$ 部分。$\delta \beta _{k+1}$ 部分的推导也可以参考文献[[2]](#[2])中“5.3.3 The error-state kinematics”公式(222b)的推导。将式(1.5)展开得到：
$$
\delta\beta _{i+1}=\delta\beta _{i}+\frac{q_{k}(a_{k}+n_{a0}-b_{a_{k}})+q_{k+1}(a_{k+1}++n_{a1}-b_{a_{k}})}{2}\delta t
$$
即，
$$
\delta\beta _{i+1}=\delta\beta _{i}+\dot{\delta\beta_{i}}\delta t
\tag{2.4}
$$
文献[2]中，公式(222b)
$$
\dot{\delta v}=-R[a_{m}-a_{b}]_{\times}\delta \theta-R\delta a_{b}+\delta g-Ra_{n}
$$
对于中值法积分下的误差状态方程为：
$$
\begin{align}
\dot{\delta\beta_{i}} =&-\frac{1}{2}q_{k}[a_{k}-b_{a_{k}}]_{\times}\delta \theta-\frac{1}{2}q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta \theta _{k+1} -\delta b_{g_{k}}\delta t+\frac{n_{w0}+n_{w1}}{2}\delta t)\delta \theta \\
 &-\frac{1}{2}q_{k}\delta b_{a_{k}}-\frac{1}{2}q_{k+1}\delta b_{a_{k}}-\frac{1}{2}q_{k}n_{a0}-\frac{1}{2}q_{k}n_{a1}
 \end{align}
\tag{2.5}
$$
将式(2.3)带入式(2.5)可得
$$
\begin{align}
\dot{\delta\beta_{i}} =&-\frac{1}{2}q_{k}[a_{k}-b_{a_{k}}]_{\times}\delta \theta-\frac{1}{2}q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}((I-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta t) \delta \theta _{k} -\delta b_{g_{k}}\delta t+\frac{n_{w0}+n_{w1}}{2}\delta t) \\
 &-\frac{1}{2}q_{k}\delta b_{a_{k}}-\frac{1}{2}q_{k+1}\delta b_{a_{k}}-\frac{1}{2}q_{k}n_{a0}-\frac{1}{2}q_{k}n_{a1}
 \end{align}
\tag{2.6}
$$
同理，可以计算出 $\delta \alpha _{k+1}$ ，可以写为：
$$
\delta\alpha _{i+1}=\delta\alpha _{i}+\dot{\delta\alpha_{i}}\delta t
\tag{2.7}
$$
$$
\begin{align}
\dot{\delta\alpha_{i}} =&-\frac{1}{4}q_{k}[a_{k}-b_{a_{k}}]_{\times}\delta \theta\delta t-\frac{1}{4}q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}((I-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta t) \delta \theta _{k} -\delta b_{g_{k}}\delta t+\frac{n_{w0}+n_{w1}}{2}\delta t)\delta t \\
 &-\frac{1}{4}q_{k}\delta b_{a_{k}}\delta t-\frac{1}{4}q_{k+1}\delta b_{a_{k}}\delta t-\frac{1}{4}q_{k}n_{a0}\delta t-\frac{1}{4}q_{k}n_{a1}\delta t
 \end{align}
\tag{2.8}
$$
最后是加速度计和陀螺仪bias的误差状态方程，
$$
\delta b_{a_{k+1}}=\delta b_{a_{k}}+n_{ba}\delta t
\tag{2.9}
$$
$$
\delta b_{w_{k+1}}=\delta b_{w_{k}}+n_{bg}\delta t
\tag{2.10}
$$
&emsp;&emsp;综合式(2.3)等误差状态方程，将其写为矩阵形式，
$$
\begin{align}
\begin{bmatrix}
\delta \alpha_{k+1}\\
\delta \theta  _{k+1}\\
\delta \beta _{k+1} \\
\delta b _{a{}{k+1}} \\
\delta b _{g{}{k+1}}
\end{bmatrix}&=\begin{bmatrix}
I & f_{01} &\delta t  & -\frac{1}{4}(q_{k}+q_{k+1})\delta t^{2} & f_{04}\\
0 & I-[\frac{w_{k+1}+w_{k}}{2}-b_{wk}]_{\times }\delta t & 0 &  0&-\delta t \\
0 &  f_{21}&I  &  -\frac{1}{2}(q_{k}+q_{k+1})\delta t & f_{24}\\
0 &  0&  0&I  &0 \\
 0& 0 & 0 & 0 & I
\end{bmatrix}
\begin{bmatrix}
\delta \alpha_{k}\\
\delta \theta  _{k}\\
\delta \beta _{k} \\
\delta b _{a{}{k}} \\
\delta b _{g{}{k}}
\end{bmatrix} \\
&+
\begin{bmatrix}
 \frac{1}{4}q_{k}\delta t^{2}&  v_{01}& \frac{1}{4}q_{k+1}\delta t^{2} & v_{03} & 0 & 0\\
 0& \frac{1}{2}\delta t & 0 & \frac{1}{2}\delta t &0  & 0\\
 \frac{1}{2}q_{k}\delta t&  v_{21}& \frac{1}{2}q_{k+1}\delta t & v_{23} & 0 & 0 \\
0 & 0 & 0 & 0 &\delta t  &0 \\
 0& 0 &0  & 0 &0  & \delta t
\end{bmatrix}
\begin{bmatrix}
n_{a0}\\
n_{w0}\\
n_{a1}\\
n_{w1}\\
n_{ba}\\
n_{bg}
\end{bmatrix}
\end{align}
\tag{2.11}
$$
其中，
$$
\begin{align}
f_{01}&=-\frac{1}{4}q_{k}[a_{k}-b_{a_{k}}]_{\times}\delta t^{2}-\frac{1}{4}q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}(I-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta t)\delta t^{2} \\
f_{21}&=-\frac{1}{2}q_{k}[a_{k}-b_{a_{k}}]_{\times}\delta t-\frac{1}{2}q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}(I-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta t)\delta t \\
f_{04}&=\frac{1}{4}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t^{2})(-\delta t) \\
f_{24}&=\frac{1}{2}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t)(-\delta t) \\
v_{01}&=\frac{1}{4}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t^{2})\frac{1}{2}\delta t \\
v_{03}&=\frac{1}{4}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t^{2})\frac{1}{2}\delta t \\
v_{21}&=\frac{1}{2}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t^{2})\frac{1}{2}\delta t \\
v_{23}&=\frac{1}{2}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t^{2})\frac{1}{2}\delta t
\end{align}
$$
将式(2.11)简写为，
$$
\delta z_{k+1} = F\delta z_{k}+VQ
$$
&emsp;&emsp;最后得到系统的雅克比矩阵 $J_{k+1}$ 和协方差矩阵 $P_{k+1}$，初始状态下的雅克比矩阵和协方差矩阵为单位阵和零矩阵，即
$$
J_{k}=I \\ P_{k}=0
$$
$$
J_{k+1}=FJ_{k}
\tag{2.12}
$$
$$
P_{k+1}=FP_{k}F^{T}+VQV_{T}
\tag{2.13}
$$

<h3 id="1">2 前端KLT跟踪 </h3>

<h3 id="1">3 系统初始化 </h3>
&emsp;&emsp;在提取的图像的Features和做完IMU的预积分之后，进入了系统的初始化环节，那么系统为什么要进行初始化呢，主要的目的有以下两个：      
- 系统使用单目相机，如果没有一个良好的尺度估计，就无法对两个传感器做进一步的融合。这个时候需要恢复出尺度；
- 要对IMU进行初始化，IMU会受到bias的影响，所以要得到IMU的bias。

所以我们要从初始化中恢复出尺度、重力、速度以及IMU的bias，因为视觉(SFM)在初始化的过程中有着较好的表现，所以在初始化的过程中主要以SFM为主，然后将IMU的预积分结果与其对齐，即可得到较好的初始化结果。      
&emsp;&emsp;系统的初始化主要包括三个环节：求取相机与IMU之间的相对旋转、相机初始化(局部滑窗内的SFM，包括没有尺度的BA)、IMU与视觉的对齐(IMU预积分中的 $\alpha$等和相机的translation)。     
####3.1 相机与IMU之间的相对旋转     
&emsp;&emsp;这个地方相当于求取相机与IMU的一部分外参。相机与IMU之间的旋转标定非常重要，偏差1-2°系统的精度就会变的极低。这部分的内容参考文献[[3]](#[3])中Ⅴ-A部分，这里做简单的总结。      
&emsp;&emsp;设相机利用对极关系得到的旋转矩阵为 $R^{c_{k}}_{c_{k+1}}$，IMU经过预积分得到的旋转矩阵为$R^{b_{k}}_{b_{k+1}}$，相机与IMU之间的相对旋转为 $R^{b}_{c}$，则对于任一帧满足，
$$
R^{b_{k}}_{b_{k+1}}R^{b}_{c}=R^{b}_{c}R^{c_{k}}_{c_{k+1}}
\tag{3.1}
$$
对式(3.1)可以做简单的证明，在其两边同乘 $^{c}x^{k+1}$ 得
$$
\begin{align}
x^{b}_{k}R^{b_{k}}_{b_{k+1}}R^{b}_{c}&=x^{b}_{k}R^{b}_{c}R^{c_{k}}_{c_{k+1}} \\
x^{b}_{k+1}R^{b}_{c}&=x^{c}_{k}R^{c_{k}}_{c_{k+1}}
\\
x^{c}_{k+1}&=x^{c}_{k+1}
\end{align}
$$
将旋转矩阵写为四元数，则式(3.1)可以写为
$$
q^{b_{k}}_{b{k+1}}\otimes q^{b}_{c}=q^{b}_{c}\otimes q^{c_{k}}_{c{k+1}}
$$
将其写为左乘和右乘的形式，综合为
$$
[Q_{1}(q^{b_{k}}_{b{k+1}})-Q_{2}(q^{c_{k}}_{c{k+1}})]q^{b}_{c}=Q^{k}_{k+1}q^{b}_{c}=0
\tag{3.2}
$$
其中 $Q_{1}(q^{b_{k}}_{b{k+1}})$，$Q_{2}(q^{c_{k}}_{c{k+1}})$ 分别表示四元数的左乘和右乘形式，
$$
\begin{align}
Q_{1}(q)&=\begin{bmatrix}
q_{w}I_{3}+[q_{xyz }]_{\times} & q_{xyz}\\
-q_{xyz} & q_{w}
\end{bmatrix} \\
Q_{2}(q)&=\begin{bmatrix}
q_{w}I_{3}-[q_{xyz }]_{\times} & q_{xyz}\\
-q_{xyz} & q_{w}
\end{bmatrix}
\end{align}
\tag{3.3}
$$
那么对于 $n$对测量值，则有
$$
\begin{bmatrix}
w^{0}_{1}Q^{0}_{1}\\
w^{1}_{2}Q^{1}_{2}\\
\vdots \\
w^{N-1}_{N}Q^{N-1}_{N}
\end{bmatrix}q^{b}_{c}=Q_{N}q^{b}_{c}=0
\tag{3.4}
$$
其中 $w^{N-1}_{N}$ 为外点剔除权重，其与相对旋转求取得残差有关，$N$为计算相对旋转需要的测量对数，其由最终的终止条件决定。残差可以写为，
$$
r^{k}_{k+1}=acos((tr(\hat{R}^{b^{-1}}_{c}R^{b_{k}^{-1}}_{b_{k+1}}\hat{R}^{b}_{c}R^{c_{k}}_{c_{k+1}} )-1)/2)
\tag{3.5}
$$
残差还是很好理解的，在具体的代码中可以计算公式(3.1)两边两个旋转的得角度差。在得到残差之后就可以进一步得到公式(3.4)中的权重，
$$
w^{k}_{k+1}=\left\{\begin{matrix}
1,\qquad r^{k}_{k+1}<threshold\\
\frac{threshold}{r^{k}_{k+1}},\qquad otherwise
\end{matrix}\right.
\tag{3.6}
$$
一般会将阈值 $threshold$ 取做 $5°$。至此，就可以通过求解方程(3.4)得到相对旋转，式(3.4)的解为 $Q_{N}$ 的左奇异向量中最小奇异值对应的特征向量。     
&emsp;&emsp;但是，在这里还要注意 __求解的终止条件(校准完成的终止条件)__ 。在足够多的旋转运动中，我们可以很好的估计出相对旋转 $R^{b}_{c}$，这时 $Q_{N}$ 对应一个准确解，且其零空间的秩为1。但是在校准的过程中，某些轴向上可能存在退化运动(如匀速运动)，这时 $Q_{N}$ 的零空间的秩会大于1。判断条件就是 $Q_{N}$ 的第二小的奇异值是否大于某个阈值，若大于则其零空间的秩为1，反之秩大于1，相对旋转$R^{b}_{c}$ 的精度不够，校准不成功。      
####3.2 相机初始化      
####3.3 视觉与IMU对齐      
&emsp;&emsp;视觉与IMU的对齐主要解决三个问题：      
&emsp;&emsp;(1) 修正陀螺仪的bias；      
&emsp;&emsp;(1) 初始化速度、重力向量 $g$和尺度因子(Metric scale)；      
&emsp;&emsp;(1) 改进重力向量 $g$的量值；      
#####3.3.1 陀螺仪Bias修正     
&emsp;&emsp;__发现校正部分使用的都是一系列的约束条件，思路很重要啊__。陀螺仪Bias校正的时候也是使用了一个简单的约束条件：
$$
\underset{\delta b_{w}}{min}\sum_{k\in B}^{ }\left \| q^{c_{0}^{-1}}_{b_{k+1}}\otimes q^{c_{0}}_{b_{k}}\otimes\gamma _{b_{k+1}}^{b_{k}} \right \|^{2}
\tag{3.7}
$$
其中
$$
\gamma _{b_{k+1}}^{b_{k}}\approx \hat{\gamma}_{b_{k+1}}^{b_{k}}\otimes \begin{bmatrix}
1\\
\frac{1}{2}J^{\gamma }_{b_{w}}\delta b_{w}
\end{bmatrix}
\tag{3.8}
$$
公式(3.7)的最小值为单位四元数 $[1,0_{v}]^{T}$ ,所以将式(3.7)进一步写为，
$$
\begin{align}
q^{c_{0}^{-1}}_{b_{k+1}}\otimes q^{c_{0}}_{b_{k}}\otimes\gamma _{b_{k+1}}^{b_{k}}&=\begin{bmatrix}
1\\
0
\end{bmatrix} \\
\hat{\gamma}_{b_{k+1}}^{b_{k}}\otimes \begin{bmatrix}
1\\
\frac{1}{2}J^{\gamma }_{b_{w}}\delta b_{w}
\end{bmatrix}&=q^{c_{0}^{-1}}_{b_{k}}\otimes q^{c_{0}}_{b_{k+1}} \\
\end{align}
$$
$$
\begin{bmatrix}
1\\
\frac{1}{2}J^{\gamma }_{b_{w}}\delta b_{w}
\end{bmatrix}=\hat{\gamma}_{b_{k+1}}^{b_{k}^{-1}}\otimes q^{c_{0}^{-1}}_{b_{k}}\otimes q^{c_{0}}_{b_{k+1}}
\tag{3.9}
$$
只取式(3.9)式虚部，在进行最小二乘求解
$$
J^{\gamma^{T}}_{b_{w}}J^{\gamma }_{b_{w}}\delta b_{w}=J^{\gamma^{T}}_{b_{w}}(\hat{\gamma}_{b_{k+1}}^{b_{k}^{-1}}\otimes q^{c_{0}^{-1}}_{b_{k}}\otimes q^{c_{0}}_{b_{k+1}}).vec
\tag{3.10}
$$
求解式(3.10)的最小二乘解，即可得到 $\delta b_{w}$，注意这个地方得到的只是Bias的变化量，需要在滑窗内累加得到Bias的准确值。    
#####3.3.2 初始化速度、重力向量 $g$和尺度因子     
&emsp;&emsp;在这个步骤中，要估计系统的速度、重力向量以及尺度因子。所以系统的状态量可以写为，
$$
X_{I}=[v^{c_{0}}_{b_{0}},v^{c_{0}}_{b_{1}},\cdots ,g^{c_{0}},s]
\tag{3.11}
$$
上面的状态量都是在 $c_{0}$ 相机坐标系下。接着，有前面的由预积分部分，IMU的测量模型可知
$$
\begin{align}
\alpha^{b_{k}}_{b_{k+1}}&=q^{b_{k}}_{c_{0}}(s(\bar{p}^{c_{0}}_{b_{k+1}}-\bar{p}^{c_{0}}_{b_{k}})+\frac{1}{2}g^{c_{0}}\triangle t_{k}^{2}-v^{c_{0}}_{b_{k}}\triangle t_{k}^{2}) \\
\beta ^{b_{k}}_{b_{k+1}}&=q^{b_{k}}_{c_{0}}(v^{c_{0}}_{b_{k+1}}+g^{c_{0}}\triangle t_{k}-v^{c_{0}}_{b_{k}})
\end{align}
\tag{3.12}
$$
在3.1小节，我们已经得到了IMU相对于相机的旋转 $q_{b}^{c}$,假设IMU到相机的平移量$p_{b}^{c}$ 那么可以很容易地将相机坐标系下的位姿转换到IMU坐标系下，
$$
\begin{align}
q_{b_{k}}^{c_{0}} &= q^{c_{0}}_{c_{k}}\otimes q_{b}^{c}  \\
s\bar{p}^{c_{0}}_{b_{k}}&=s\bar{p}^{c_{0}}_{c_{k}}+q^{c_{0}}_{c_{k}}p_{b}^{c}
\end{align}
\tag{3.13}
$$
综合式(3.12)和式(3.13)可得，
$$
\begin{align}
\hat{z}^{b_{k}}_{b_{k+1}}&=\begin{bmatrix}
\alpha^{b_{k}}_{b_{k+1}}-q^{c_{0}}_{c_{k+1}}p^{c}_{b}+q^{c_{0}}_{c_{k}}p^{c}_{b}&\\
\beta ^{b_{k}}_{b_{k+1}}
\end{bmatrix}=H^{b_{k}}_{b_{k+1}}X_{I}+n^{b_{k}}_{b_{k+1}} \\
&\approx \begin{bmatrix}
-q^{c_{0}}_{b_{k}}\triangle t_{k} &0&  1/2q^{c_{0}}_{b_{k}}\triangle t_{k}^{2} &q^{c_{0}}_{b_{k}}(\bar{p}^{c_{0}}_{b_{k+1}}-\bar{p}^{c_{0}}_{b_{k}}) \\
 -q^{c_{0}}_{b_{k}}& q^{c_{0}}_{b_{k}} &q^{c_{0}}_{b_{k}}\triangle t_{k}   & 0
\end{bmatrix}\begin{bmatrix}
v^{c_{0}}_{b_{k}}\\
v^{c_{0}}_{b_{k+!}}\\
g^{c_{0}}\\
s
\end{bmatrix}
\end{align}
$$
最后求解最小二乘问题
$$
\underset{\delta b_{w}}{min}\sum_{k\in B}^{ }\left \|
\hat{z}^{b_{k}}_{b_{k+1}}-H^{b_{k}}_{b_{k+1}}X_{I}
 \right \|^{2}
$$
至此即可求解出所有状态量，但是对于重力向量 $g^{c_{0}}$ 还要做进一步的纠正。在纠正$g^{c_{0}}$ 的过程中，会对速度也做进一步的优化。      
#####3.3.3 纠正重力向量    

<h3 id="1">4 系统边缘化 </h3>
&emsp;&emsp;在Sliding Window做优化的过程中，滑窗内的pose和feature个数是有限的，在系统优化的过程只，势必要不断将一些pose和feature移除滑窗，但是无故地移除这些pose和feature会丢弃某些约束，进而降低了优化的精度，所以在移除pose和feature的时候需要将相关联的约束转变为一个先验的约束条项作为prior放到优化问题中，以上就可以描述为marginalization(边缘化)的过程。

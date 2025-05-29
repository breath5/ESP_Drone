
# 一.ESP32S3-Drone（遥控器位于RC分支）

## 1. 器件

### 1.1 主控

**ESP32S3**

### 1.2 传感器

#### 1.2.1 IMU传感器

**MPU6050**或MPU9250

用于测量飞行器的姿态和加速度


用于测量飞行器的高度

### 1.3 电源模块

#### 1.3.1 电源

**1s动力电池（长度小于65，宽小于17.5，厚小于7.5，3.7V）**

接头类型最好选择PH2.0容易插拔

#### 1.3.2 升压和减压模块

**ps7516,AMS1117**

电源是先通过ps7516芯片，从电池电压3.7v-4.2v升压成5v再通过AMS1117-3.3减压成3.3v主要起到一个缓冲的作用

#### 1.3.3 电压测量

一个简单的电阻分压电路

### 1.4 电机及控制

#### 1.4.1 电机选型

**8520空心杯电机**

注意：电机的轴径分1.0mm和1.2mm，要和螺旋桨的内径相同不然会装不上，就算硬怼上了螺旋桨也会偏心。

#### 1.4.2 电机控制

**SI2302mos管**

### 1.5 其它外设

外壳，螺旋桨，减震圈

# 二.环境搭建

## 1.ESP-IDF

### 1.1esp-idf版本

### esp-idf v4.4.5

# 三.姿态解算

## 1.mpu6050原始数据

126

## 2.零偏校准

134

## 3.原始数据滤波

198

## 4.四元数姿态解算

179

## 5.级联PID

195

![屏幕截图 2024-08-29 174256](C:\Users\Lenovo\OneDrive\图片\屏幕快照\屏幕截图 2024-08-29 174256.png)

```c
    PID_Postion_Cal(&PID_ROL_Angle,Target_Angle.roll,Measure_Angle.roll);//ROLL角度环PID （输入角度 输出角速度）
    PID_Postion_Cal(&PID_PIT_Angle,Target_Angle.pitch,Measure_Angle.pitch);//PITH角度环PID （输入角度 输出角速度）
    PID_Postion_Cal(&PID_YAW_Angle,Target_Angle.yaw,Measure_Angle.yaw);//YAW角度环PID  （输入角度 输出角速度）

	//角速度环
	PID_Postion_Cal(&PID_ROL_Rate,PID_ROL_Angle.OutPut,(gyr_in->gyro_f.Y*RadtoDeg)); //ROLL角速度环PID （输入角度环的输出，输出电机控制量）
	PID_Postion_Cal(&PID_PIT_Rate,PID_PIT_Angle.OutPut,(gyr_in->gyro_f.X*RadtoDeg)); //PITH角速度环PID （输入角度环的输出，输出电机控制量）
	PID_Postion_Cal(&PID_YAW_Rate,PID_YAW_Angle.OutPut,(gyr_in->gyro_f.Z*RadtoDeg)); //YAW角速度环PID （输入角度，输出电机控制量）
```



==PID代码：==

```c
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
{

	PID->Error  = target - measure;              //误差
	PID->Differ = PID->Error - PID->PreError;    //微分量

	PID->Pout = PID->P * PID->Error;                        //比例控制
	PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;  //积分控制
	PID->Dout = PID->D * PID->Differ;                       //微分控制

	PID->OutPut =  PID->Pout + PID->Iout + PID->Dout;       //比例 + 积分 + 微分总控制

	if(state.isRCLocked == true && setpoint.thrust >= 180)    //飞机解锁之后再加入积分,防止积分过调
		{
			if(measure > (PID->Ilimit)||measure < -PID->Ilimit)   //积分分离
			{PID->Ilimit_flag = 0;}
			else
			{
				PID->Ilimit_flag = 1;                               //加入积分(只有测量值在-PID->Ilimit~PID->Ilimit 范围内时才加入积分)
				PID->Integral += PID->Error;                        //对误差进行积分
				if(PID->Integral > PID->Irang)                      //积分限幅
					PID->Integral = PID->Irang;
				if(PID->Integral < -PID->Irang)                     //积分限幅
				    PID->Integral = -PID->Irang;
			}
		}else
		{PID->Integral = 0;}

	PID->PreError = PID->Error ;                            //前一个误差值
}
```



### 1.积分项开启

```c
if (state.isRCLocked == true && setpoint.thrust >= 180)
{
    ......
}
else { PID->Integral = 0; }
```

`setpoint.thrust >= 180` 确保油门达到一定的阈值（180），这样只有在无人机解锁并准备飞行时才会启用 PID 控制中的积分部分。**这可以防止在未起飞或油门较低时积分累积，导致积分过调。*如果无人机未解锁或油门值低于 180，程序会将积分项清零，防止积分在无人机未起飞时积累。**【在无人机解锁之前或油门值很小时，控制系统的误差可能较大，如果此时启用积分控制，可能会积累过多的积分，导致起飞后或飞行过程中系统反应过度。通过在解锁后且油门达到一定阈值时才启用积分，可以减少这种情况，**提升控制系统的稳定性**。】

### 2.积分分离

```c
if (measure > PID->Ilimit || measure < -PID->Ilimit)
```

当测量值 `measure` 超过限制范围 `PID->Ilimit` 时，`PID->Ilimit_flag` 设置为 `0`，禁用积分。否则，`PID->Ilimit_flag` 设置为 `1`，允许积分生效。这种方式**确保积分只在测量值处于合理范围内时进行**，**以防止积分对大误差的情况做出不合理的调整**。

### 3.积分限幅

```c
if(PID->Integral > PID->Irang)                      //积分限幅
{
  PID->Integral = PID->Irang; 
}

if(PID->Integral < -PID->Irang)                     //积分限幅
{
   PID->Integral = -PID->Irang;
}
```

`PID->Integral` 的值通过 `PID->Irang` 被限制在一定范围内。**这样可以防止积分累积过多，导致系统不稳定**。

### 小结

1.**避免过调和积分饱和**：在控制过程中，如果误差长时间存在，积分项会不断累积。当误差突然减小时，由于积分项积累过多，会导致系统响应过度，即积分过调，导致系统不稳定，通过积分分离，只有当误差或测量值在合理范围内时，才会启用积分控制，防止积分项过大。

2.**减小积分项对大误差的影响**：在误差较大时，积分项的累积会导致控制输出不稳定，而积分分离可以在误差较大的情况下禁用积分，从而确保系统能够对较大误差做出快速响应，而不被积分项拖累。



## 6.动力分配

210

电机分配的基本思想是每个电机等于==三个自由度的加减的结合==，根据这三个自由度变化时所期望的==电机速度的增减来确定具体是增还是减==。

```c
             Moto_PWM_1 = rc_in->thrust + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
			Moto_PWM_2 = rc_in->thrust - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
			Moto_PWM_3 = rc_in->thrust - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
			Moto_PWM_4 = rc_in->thrust + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
```

1. **Moto_PWM_1 = rc_in->thrust + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;**

- **位置**：电机1位于前左（逆时针旋转）。

- 分析

  ：

  - `+ PID_ROL_Rate.OutPut`：增加横滚角，使无人机向右倾斜。
  - `+ PID_PIT_Rate.OutPut`：增加俯仰角，使无人机向前倾斜。
  - `- PID_YAW_Rate.OutPut`：减小偏航角，抵消逆时针旋转的趋势。

- **效果**：控制无人机向前右方向移动并稳定偏航。

2. **Moto_PWM_2 = rc_in->thrust - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;**

- **位置**：电机2位于前右（顺时针旋转）。

- 分析

  ：

  - `- PID_ROL_Rate.OutPut`：减小横滚角，使无人机向左倾斜。
  - `+ PID_PIT_Rate.OutPut`：增加俯仰角，使无人机向前倾斜。
  - `+ PID_YAW_Rate.OutPut`：增加偏航角，增强顺时针旋转的趋势。

- **效果**：控制无人机向前左方向移动并强化偏航。

3. **Moto_PWM_3 = rc_in->thrust - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;**

- **位置**：电机3位于后右（逆时针旋转）。

- 分析

  ：

  - `- PID_ROL_Rate.OutPut`：减小横滚角，使无人机向左倾斜。
  - `- PID_PIT_Rate.OutPut`：减小俯仰角，使无人机向后倾斜。
  - `- PID_YAW_Rate.OutPut`：减小偏航角，抵消逆时针旋转的趋势。

- **效果**：控制无人机向后左方向移动并稳定偏航。

4. **Moto_PWM_4 = rc_in->thrust + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;**

- **位置**：电机4位于后左（顺时针旋转）。

- 分析

  ：

  - `+ PID_ROL_Rate.OutPut`：增加横滚角，使无人机向右倾斜。
  - `- PID_PIT_Rate.OutPut`：减小俯仰角，使无人机向后倾斜。
  - `+ PID_YAW_Rate.OutPut`：增加偏航角，增强顺时针旋转的趋势。

- **效果**：控制无人机向后右方向移动并强化偏航。

### **总结**

- **横滚控制（Roll）**：通过`PID_ROL_Rate.OutPut`控制左右电机之间的推力差异，使无人机左右倾斜。
- **俯仰控制（Pitch）**：通过`PID_PIT_Rate.OutPut`控制前后电机之间的推力差异，使无人机前后倾斜。
- **偏航控制（Yaw）**：通过`PID_YAW_Rate.OutPut`控制相对旋转方向不同的电机组的推力差异，实现无人机的旋转。

这些组合通过调节各个电机的推力，能够实现无人机在空间中各个方向的运动和姿态控制，同时保持稳定。

# 四.姿态解算详细

## 1.使用方法

1. **互补滤波**：
   - 文件中定义了互补滤波的两个权重 `Kp_New`（0.9）和 `Kp_Old`（0.1），用于对当前数据和历史数据进行加权平均。这种方法通常用于结合加速度计和陀螺仪的数据进行姿态估计。
   ```c
   #define Kp_New 0.9f
   #define Kp_Old 0.1f
   ```

2. **四元数融合算法**：
   
   - 文件中实现了基于四元数的姿态解算算法，通过融合加速度计和陀螺仪的数据，来计算无人机的姿态（滚转角、俯仰角、航向角）。
   ```c
   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
   ```
   
3. **方向余弦矩阵**：
   - 文件中计算了方向余弦矩阵（DCM），用于将惯性坐标系转换到机体坐标系。
   ```c
   matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;
   matrix[1] = 2.f * (q1q2 + q0q3);
   matrix[2] = 2.f * (q1q3 - q0q2);
   matrix[3] = 2.f * (q1q2 - q0q3);
   matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;
   matrix[5] = 2.f * (q2q3 + q0q1);
   matrix[6] = 2.f * (q1q3 + q0q2);
   matrix[7] = 2.f * (q2q3 - q0q1);
   matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;
   ```

## 2.主要步骤

1. **数据滤波**：
   
   - 使用快速排序和窗口滑动滤波函数对加速度数据进行滤波处理，去除噪声。
   ```c
   void SortAver_FilterXYZ(INT16_XYZ *acc, FLOAT_XYZ *Acc_filt, uint8_t n)
   ```
   
2. **重力向量估计**：
   - 使用陀螺仪积分估计重力向量。
   ```c
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;
   ```

3. **误差计算和补偿**：
   - 计算测量的重力向量与估算的重力向量的误差，并进行积分补偿。
   ```c
   ex = (ay*vz - az*vy);
   ey = (az*vx - ax*vz);
   ez = (ax*vy - ay*vx);
   ```

4. **姿态解算**：
   - 使用四元数微分方程更新姿态，并将四元数单位化。
   ```c
   q0 = q0 * norm;
   q1 = q1 * norm;
   q2 = q2 * norm;
   q3 = q3 * norm;
   ```

5. **欧拉角转换**：
   
   - 将四元数转换成欧拉角，以获得滚转角、俯仰角和航向角。
   ```c
   stateimu->attitude.roll = -asin(2.f * (q1q3 - q0q2))* 57.3f;
   stateimu->attitude.pitch = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f;
   ```

1弧度 ≈ 57.2958 度

```
1 radian=180/π≈57.2958 degrees
```



## 3.结论

主要通过**互补滤波和四元数融合算法**进行无人机姿态解算，结合加速度计和陀螺仪的数据，最终计算出无人机的滚转角、俯仰角和航向角。

## 4.卡尔曼滤波

为了在 `IMU.c` 文件中加入卡尔曼滤波，需要以下几个步骤：

1. 定义卡尔曼滤波器的状态变量和协方差矩阵。
2. 编写初始化卡尔曼滤波器的函数。
3. 编写卡尔曼滤波更新的函数。
4. 在姿态解算过程中使用卡尔曼滤波。

以下是具体的实现步骤和示例代码：

### 1. 定义卡尔曼滤波器的状态变量和协方差矩阵

```c
// 状态向量（姿态角度：滚转角、俯仰角、航向角）
typedef struct {
    float roll;
    float pitch;
    float yaw;
} KalmanState;

// 卡尔曼滤波器的结构体
typedef struct {
    KalmanState state;      // 状态估计
    float P[3][3];          // 状态估计的误差协方差矩阵
    float Q[3];             // 过程噪声协方差
    float R[3];             // 测量噪声协方差
} KalmanFilter;
```

### 2. 初始化卡尔曼滤波器的函数

```c
void KalmanFilter_Init(KalmanFilter *kf) {
    // 初始化状态
    kf->state.roll =  0;
    kf->state.pitch = 0;
    kf->state.yaw =   0;
    
    // 初始化误差协方差矩阵P
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            kf->P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // 设置过程噪声和测量噪声协方差
    kf->Q[0] = 0.01f; // 滚转角的过程噪声
    kf->Q[1] = 0.01f; // 俯仰角的过程噪声
    kf->Q[2] = 0.01f; // 航向角的过程噪声
    kf->R[0] = 0.1f;  // 滚转角的测量噪声
    kf->R[1] = 0.1f;  // 俯仰角的测量噪声
    kf->R[2] = 0.1f;  // 航向角的测量噪声
}
```

### 3. 卡尔曼滤波更新函数

```c
void KalmanFilter_Update(KalmanFilter *kf, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    // 预测步骤
    kf->state.roll += gx * dt;
    kf->state.pitch += gy * dt;
    kf->state.yaw += gz * dt;

    for (int i = 0; i < 3; i++) {
        kf->P[i][i] += kf->Q[i];
    }

    // 计算卡尔曼增益
    float K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = kf->P[i][i] / (kf->P[i][i] + kf->R[i]);
    }

    // 计算测量误差
    float roll_measured = atan2(ay, az) * 57.2958f; // 将弧度转换为度
    float pitch_measured = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958f;

    float y[3];
    y[0] = roll_measured - kf->state.roll;
    y[1] = pitch_measured - kf->state.pitch;
    y[2] = 0 - kf->state.yaw; // 简单处理航向角为0

    // 更新状态估计
    kf->state.roll += K[0] * y[0];
    kf->state.pitch += K[1] * y[1];
    kf->state.yaw += K[2] * y[2];

    // 更新误差协方差矩阵
    for (int i = 0; i < 3; i++) {
        kf->P[i][i] *= (1 - K[i]);
    }
}
```

### 4. 在姿态解算过程中使用卡尔曼滤波

在姿态解算函数中，使用上述卡尔曼滤波更新函数：

```c
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    static KalmanFilter kf;
    static int is_initialized = 0;
    
    if (!is_initialized) {
        KalmanFilter_Init(&kf);
        is_initialized = 1;
    }

    // 更新卡尔曼滤波器
    KalmanFilter_Update(&kf, gx, gy, gz, ax, ay, az, dt);

    // 获取姿态角度
    stateimu->attitude.roll = kf.state.roll;
    stateimu->attitude.pitch = kf.state.pitch;
    stateimu->attitude.yaw = kf.state.yaw;
}
```

结论

通过上述步骤，将卡尔曼滤波添加到 `IMU.c` 文件中，可以更准确地进行姿态解算。卡尔曼滤波器通过结合预测模型和测量数据，能够有效地滤除噪声，提高姿态角的估计精度。

🌐 Sources

1. [csdn.net - Kalman Filter 通俗讲解原创](https://blog.csdn.net/u010665216/article/details/80556000)
2. [csdn.net - 信号采样基本概念—— 6. 卡尔曼滤波（Kalman Filtering） 原创](https://blog.csdn.net/poisonchry/article/details/131820298)
3. [cnblogs.com - 面向软件工程师的卡尔曼滤波器- 人工智能遇见磐创](https://www.cnblogs.com/panchuangai/p/12567860.html)
4. [kalmanfilter.net - 卡尔曼滤波教程](https://www.kalmanfilter.net/CN/default_cn.aspx)
5. [admin.guyuehome.com - 22. 卡尔曼滤波器详解——从零开始(2) Kalman Filter from Zero](http://admin.guyuehome.com/36218)
6. [cnblogs.com - 关于卡尔曼滤波中协方差矩阵Q,R的一些思考,卡尔曼原理讲解](https://www.cnblogs.com/laozhu1234/p/14932627.html)

## 5.互补滤波

  由于从陀螺仪的角速度获得角度信息，需要经过积分运算。如果角速度信号**存在微小偏差，经过积分运算之后，变化形成积累误差**。这个误差会随时间延长逐步增加，最终导致饱和，无法得到正确的姿态角度信号。因此对于四元数的龙格库塔法或者方向余弦的微分选代中使用的角速度信号都需要**对这个累积误差进行消除处理。**==加速度计的特点是噪声大，但是不会随时间累积误差，虽然没有漂移，但数据容易受短时噪声影响。陀螺仪用于测量角速度，数据更新快，但由于存在漂移，长时间运行时误差会累积。==互补滤波结合了**低频段的加速度计数据和高频段的陀螺仪数据**来获得稳定的姿态信息。

==核心思想==：陀螺仪提供短时间内快速变化的姿态信息，准确跟踪物体的快速运动，加速度计提供长期稳定的信息，用于修正陀螺仪漂移。通过加权（比例项和积分项），两者的输出数据被融合，从而获得既稳定又响应迅速的姿态估计。



```c
#define Kp 1.1f   // 比例增益，控制加速度计和磁力计数据的收敛速率
#define Ki 0.001f // 积分增益，控制陀螺仪偏差的收敛速率
#define halfT 0.005f // 采样周期的一半，5ms的意思
```

- **`Kp` 是比例增益**，调节加速度计的修正速率，数值越大，陀螺仪的响应越快，但容易引入噪声。
- **`Ki` 是积分增益**，调节陀螺仪漂移的收敛速率。

```c
exInt = exInt + ex * Ki;
eyInt = eyInt + ey * Ki;
ezInt = ezInt + ez * Ki;
```

`ex`, `ey`, `ez` 是姿态误差，表示陀螺仪测量之间的差异。将误差累积到积分项 `exInt`, `eyInt`, `ezInt` 中，这样可以==逐渐校正陀螺仪的漂移==。

![屏幕截图 2024-09-09 191814](E:\Data\TyporaData\TyporaPhoto\屏幕截图 2024-09-09 191814.png)

```c
gx = gx + Kp*ex + exInt;
gy = gy + Kp*ey + eyInt;
gz = gz + Kp*ez + ezInt;
```

通过比例项 `Kp*ex` 和积分项 `exInt` 将**误差补偿到陀螺仪的角速度** (`gx`, `gy`, `gz`) 上，从而实现校正陀螺仪漂移的效果。

![屏幕截图 2024-09-09 191830](E:\Data\TyporaData\TyporaPhoto\屏幕截图 2024-09-09 191830.png)

```c
q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
q1 = q1 + (q0*gx + q2*gz - q3*gy)  * halfT;
q2 = q2 + (q0*gy - q1*gz + q3*gx)  * halfT;
q3 = q3 + (q0*gz + q1*gy - q2*gx)  * halfT;
```

基于四元数的姿态更新方程，利用补偿后的陀螺仪数据来更新四元数 `q0`, `q1`, `q2`, `q3`。四元数能够表示物体的三维旋转姿态，并避免万向节锁死问题。

## 6.**窗口滑动平均滤波算法**(移动平滑滤波)

由于飞行器电机马达和桨叶挥舞产生的机==体振动较大==，频率较高，MEMS传感器的原始数据会受到该振动的影响，从而输出带有高频噪声的原始数据，如图7-6（a）所示。这种高频噪声对于后续的姿态解算以及飞控算法都有一定的损害，因此必须将其滤除。一个比较常用的简单滤波方法是==平滑滤波==。

![屏幕截图 2024-09-09 175738](D:\TyporaData\TyporaPhoto\屏幕截图 2024-09-09 175738.png)



（a）中带有噪声的传感器原始信号在时域内所呈现的**短时间跳变的起伏不平的现象**转换到频域范围内则代表了高频成分，**上升*和下降的速度越快，则表示频率越高**。**不随时间变化或者随时间缓慢变化的部分则是频率较低的信号**。所谓平滑滤波就是==指使这些不平滑的高频成分变得平滑，使得变化没有原来那么剧烈，整个信号更能反映原信号低频分量的成分==。因此平滑滤波实际上是一**种低通滤波**
**器**，和其他滤波器不同的是，它是一种从==时域方面进行设计的滤波算法==，属于**低频增强的时域滤波技术**，有的图像处理技术也通过空间域平滑滤波的算法进行降噪处理。

![屏幕截图 2024-09-09 180741](D:\TyporaData\TyporaPhoto\屏幕截图 2024-09-09 180741.png)

平滑滤波一般采用==时域或空间域的简单平均法==的方式实现，也就是**对时域或空间域相邻近的采样点的数值求平均来去除突变的数据**。**因此邻近采样点的数值大小偏差直接影响平滑滤波的效果，邻域的数值相差越大，则平滑效果越好，但高频信息的损失也越大。平滑滤波的思想可以用图7-5来表示。**

![屏幕截图 2024-09-09 180700](E:\Data\TyporaData\TyporaPhoto\屏幕截图 2024-09-09 180700.png)

```c
void  SortAver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t n){ //窗口滑动滤波函数
  static float bufx[20],bufy[20],bufz[20];   //用于存储 X、Y、Z 方向上的传感器数据
  static uint8_t cnt =0,flag = 1;
  float temp1=0,temp2=0,temp3=0;
  uint8_t i;
  bufx[cnt] = acc->X;
  bufy[cnt] = acc->Y;
  bufz[cnt] = acc->Z;
  cnt++;               //这个的位置必须在赋值语句后，否则bufx[0]不会被赋值
    
 /*当 cnt 的值小于 n 时，说明滑动窗口还没有填满，程序直接返回，不做进一步计算。当窗口填满后，flag 设置为 0，表示后续将始终进行计算*/
  if(cnt<n && flag)    //如果两个条件都不为零则条件满足  【判断窗口是否填满】
    return;            //数组填不满不计算
  else
    flag = 0;
 
    /*使用 QuiteSort 函数对 bufx[], bufy[], 和 bufz[] 进行排序，排序后，取除去第一个和最后一个元素（极值）的中间部分进行平均计算*/
  QuiteSort(bufx,0,n-1);
  QuiteSort(bufy,0,n-1);
  QuiteSort(bufz,0,n-1);
  for(i=1;i<n-1;i++)
   {
    temp1 += bufx[i];
    temp2 += bufy[i];
    temp3 += bufz[i];
   }

    /*对去除极值后的数据进行累加，并除以 n-2 以得到滑动窗口的平均值*/
   if(cnt>=n) cnt = 0;
   Acc_filt->X  = temp1/(n-2);
   Acc_filt->Y  = temp2/(n-2);
   Acc_filt->Z  = temp3/(n-2);
}
```

# 五.驱动显示屏

## 1.镜像

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/0fd3d135dd752235462b6ae313e762ff.png)

## 2.RC-UI



# 六.飞控相关知识点

## 1.欧拉角

![img](https://i-blog.csdnimg.cn/blog_migrate/d8f213581e17c6691cbe33f0285eeed2.png)

pitch()：俯仰，将物体绕Y轴旋转（localRotationY）

![img](https://i-blog.csdnimg.cn/blog_migrate/39b49c1ab1c2093975194cf7c143ce83.gif)

roll()：横滚，将物体绕X轴旋转（localRotationX）

![pitch yaw roll 的区别（转载） - 江南雨 - 江南雨的博客](https://i-blog.csdnimg.cn/blog_migrate/a9fe44cfcf9715fa7bf377cfab7026cc.gif)

yaw()：航向，将物体绕Z轴旋转（localRotationZ）

![pitch yaw roll 的区别（转载） - 江南雨 - 江南雨的博客](https://i-blog.csdnimg.cn/blog_migrate/cf6d7ae78c1757fee9928497d08ff774.gif)

## 2.飞行运动

根据牛顿[第三定律](https://zhida.zhihu.com/search?q=第三定律&zhida_source=entity&is_preview=1)，旋翼在旋转的同时，也会同时向电机施加一个**反作用力（反扭矩）**，**促使电机向反方向旋转**。所以为了避免飞机疯狂自旋，四[旋翼飞机](https://zhida.zhihu.com/search?q=旋翼飞机&zhida_source=entity&is_preview=1)的四个螺旋桨中，**相邻的两个螺旋桨旋转方向是相反的**。

### 1.垂直运动

![img](https://i-blog.csdnimg.cn/blog_migrate/36cfd16b083b0b67b0a022936c025cb0.png)

如上图所示，在保证四旋翼无人机**每个旋翼转速大小相同**（==产生的总扭矩为零==），对每个旋翼**增加/减小大小相同的转速**，便可实现无人机的垂直上升/下降运动。

### 2.原地旋转（与下面的偏航运动重复）

当要飞机**原地旋转时**，我们就可以==利用这种反扭矩==，M2、M4两个顺时针旋转的电机转速增加，M1、M3号两个[逆时针旋转](https://zhida.zhihu.com/search?q=逆时针旋转&zhida_source=entity&is_preview=1)的电机转速降低，由于反扭矩影响，飞机就会产生逆时针方向的旋转。

![img](https://pica.zhimg.com/80/v2-e0448d852bd1f64dfda655f550852f42_720w.webp)

### 3.俯仰运动（水平运动）

![img](https://i-blog.csdnimg.cn/blog_migrate/8229d9a41568d1f0f53ba137b1b6f997.png)

将电机**Motor1**、**Motor4**的转速减小或者将**Motor2**、**Motor3**增加时，四旋翼会产生向前上方的力，使四旋翼向前飞行。反之，如果将**Motor1**、**Motor4**的转速增加或者将**Motor2**、**Motor3**减小时，四旋翼会产生向后上方的力，使四旋翼向后飞行。

倾斜时的侧面平视如下图，这时螺旋桨产生的升力除了在竖直方向上抵消飞机重力外，还在水平方向上有一个分力，这个分力就让飞机有了水平方向上的加速度，飞机也因而能向前飞行。

![img](https://pic2.zhimg.com/80/v2-fbd6b37d5e904cdcd075050f68905d85_720w.webp)

### 4.滚转运动

![img](https://i-blog.csdnimg.cn/blog_migrate/21cf235bdecdc5e0cbcb6ad190dac217.png)

将电机**Motor1**、**Motor2**的转速增加或者将**Motor3**、**Motor4**的转速减小时，四旋翼会产生向右上方的合力，使四旋翼向右飞行。反之，如果减小**Motor1**、**Motor2**的转速或者增加**Motor3**、**Motor4**的转速，四旋翼会产生向左上方的合力，使四旋翼向左飞行。

### 5.偏航运动

![img](https://i-blog.csdnimg.cn/blog_migrate/da96b1e69a568dfa8a359ef169f2683c.png)

将电机**Motor2**、**Motor4**的转速增加或者将**Motor1**、**Motor3**的转速减小，四旋翼会向右旋转，实现向右偏航。反之，如果将**Motor2**、**Motor4**的转速减小或者将**Motor1**、**Motor3**的转速增加，四旋翼会向左旋转，实现向左偏航。

## 3.PID调参

先从内环的P开始，调整到震荡再加一点D抑制，然后再外环，I一般是后面感觉有偏差再加，并且不要把I加太大，I太大会发抖。

从抖动和震荡，加D多少算是抑制比较好，靠感悟。

# 七.RC与Drone通信（UDP）

[![img](https://subingwen.cn/linux/udp/udp.jpg)](https://subingwen.cn/linux/udp/udp.jpg)

## 1.服务器端

1.创建套接字节

```c
// 第二个参数是 SOCK_DGRAM（UDP类型）, sock = socket(AF_INET, SOCK_DGRAM, 0)，第三个参数0表示使用报式协议中的udp
/*1.地址族，2.套接字节类型。通用指定协议，AF_INET为IPv4地址*/
int fd = socket(AF_INET, SOCK_DGRAM, 0);
```

2.使用通信的套接字和本地的IP和端口绑定

```c
bind(sock, (struct sockaddr *) &server_addr, sizeof(server_addr))；
```

3.通信

```c
//接收数据
recvfrom();
//发送数据
sendto();
```

```c
 recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &addr_len);
 sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
```

4.关闭套接字节

```c
close(sock);
```

## 2.客户端

客户端没有上面的绑定操作，因为客户端不需要和服务端绑定，服务端会向指定端口发送数据

## 3.通信函数

基于UDP进行套接字通信，创建套接字的函数还是`socket()`但是第二个参数的值需要指定为`SOCK_DGRAM`，通过该参数指定要创建一个基于报式传输协议的套接字，最后一个参数指定为0表示使用报式协议中的UDP协议。

```c
int socket(int domain, int type, int protocol);
```

- 参数:
  - domain：地址族协议，AF_INET -> IPv4，AF_INET6-> IPv6
  - type：使用的传输协议类型，报式传输协议需要指定为 SOCK_DGRAM
  - protocol：指定为0，表示使用的默认报式传输协议为 UDP
- 返回值：函数调用成功返回一个可用的文件描述符（大于0），调用失败返回-1

另外进行UDP通信，通信过程虽然默认还是阻塞的，但是通信函数和TCP不同，操作函数原型如下：

```c

// 接收数据, 如果没有数据,该函数阻塞
size_t recvfrom(int sockfd, void *buf, size_t len, int flags,
                 struct sockaddr *src_addr, socklen_t *addrlen);
```

- 参数:
  - sockfd: 基于udp的通信的文件描述符
  - buf: 指针指向的地址用来存储接收的数据
  - len: buf指针指向的内存的容量, 最多能存储多少字节
  - flags: 设置套接字属性，一般使用默认属性，指定为0即可
  - src_addr: 发送数据的一端的地址信息，IP和端口都存储在这里边, 是大端存储的
    - 如果这个参数中的信息对当前业务处理没有用处, 可以指定为NULL, 不保存这些信息
  - addrlen: 类似于accept() 函数的最后一个参数, 是一个传入传出参数
    - 传入的是src_addr参数指向的内存的大小, 传出的也是这块内存的大小
    - 如果src_addr参数指定为NULL, 这个参数也指定为NULL即可
- 返回值：成功返回接收的字节数，失败返回-1

```c

// 发送数据函数
ssize_t sendto(int sockfd, const void *buf, size_t len, int flags, const struct sockaddr *dest_addr, socklen_t addrlen);
```

- 参数:
  - sockfd: 基于udp的通信的文件描述符
  - buf: 这个指针指向的内存中存储了要发送的数据
  - len: 要发送的数据的实际长度
  - flags: 设置套接字属性，一般使用默认属性，指定为0即可
  - dest_addr: 接收数据的一端对应的地址信息, 大端的IP和端口
  - addrlen: 参数 dest_addr 指向的内存大小
- 返回值：函数调用成功返回实际发送的字节数，调用失败返回-1

# 八.通信协议

查看上位机文档

# 九.任务设置

## 1.Drone

```c
/*anotc__client校准数据模块*/
xTaskCreate(anotc_send,"anotc_send",1024*5,NULL,4, NULL );//校准完成后反馈信息给匿名上位机

/*remote_control遥控器模块*/
xTaskCreate(remote_control_task,"remote_control_task",4096,NULL,6,NULL);//循环向遥控器反馈数据

/*UDP_TCP模块，与地面站通信*/
xTaskCreate(udp_anotc_read_task, "udp_anotc_read_task", 1024*8, (void*)AF_INET, 5, NULL);//创建接收地面站数据的任务
xTaskCreate(udp_rc_read_task, "udp_rc_read_task", 1024*8, (void*)AF_INET, 8, NULL); //创建接收遥控器数据的任务

/*control飞行控制模块*/
xTaskCreate(angle_control_Task, "angle_control_Task", 1024 * 8, NULL, 24, NULL);

/*LED模块*/
xTaskCreate(led_Task, "led_Task", 1024 * 5, NULL, 0, NULL); //创建任务，判断状态改变灯的亮灭

/*VBAT电压模块*/
xTaskCreate(VBAT_task, "VBAT_task", 1024*2, NULL, 0, NULL);//创建任务不断循环获取电池电压
```

优先级：control -> remote_control  -> UDP_TCP_rc  ->  UDP_TCP_anotc  -> anotc__client -> LED，VBAT

## 2.RC

```c
/*RC_contrl_signal模块*/
xTaskCreate(control_signal_task, "control_signal_task", 1024*4, NULL, 3, NULL);  //启用信号获取任务，循环获取摇杆与按钮的信号

 /*lvgl模块*/
xTaskCreatePinnedToCore(lv_task, "lv_task", 1024 * 8, NULL, 5, NULL, 1); // 在CPU1创建lvgl任务

/*LED任务*/
xTaskCreate(led_Task, "led_Task", 1024 * 5, NULL, 8, NULL);  //电量灯

/*UDP_TCP模块*/
 xTaskCreate(udp_client_task, "udp_client", 4096*2, NULL, 5, NULL);  //接收Drone数据

/*remote_control模块*/
xTaskCreate(remote_send_task, "remote_send_task", 1024*2, NULL, 3, NULL);  //发送控制信号
```

# 十.Drone和RC信号处理

整体逻辑是将遥控器的输入信号经过校正和缩放处理后，转换成具体的目标姿态角度，以便于后续的姿态控制算法使用这些目标值进行姿态调节。【遥控器输出的信号输出到Drone才进行处理，减少干扰，减低复杂度】

==在Drone中将遥控器中立值补偿到1500，再将信号转换为实际角度值（包含零点偏移和范围映射）。==

## 1.航向调整（Yaw）

```c
if(Target_Angle.yaw >1 || Target_Angle.yaw <-1)
{
     rc_yaw -= Target_Angle.yaw/100;
}
```

如果目标偏航角度的绝对值超过 `1`（即超过 ±1°），就认为偏航角度偏离了中立位置，表明遥控器发出了偏航指令，飞机需要调整偏航角。程序会调整 `rc_yaw`，并且调整的幅度是 `Target_Angle.yaw / 100`。这个除以 `100` 的操作会使**调整过程较为平滑**，==避免偏航角度的突然剧烈变化==。通过逐步减少 `rc_yaw`，飞行器的偏航角会逐步向目标值靠近，避免突兀的动作，**增加飞行的平稳性和可控性**。

# 十一.lvgl架构

## 1.架构层次

LVGL 主要分为三层：

**硬件抽象层 (HAL)**：负责与底层硬件（如显示设备、输入设备、存储等）的交互。

**核心模块**：负责 GUI 元素的创建、事件处理、动画等功能。

**应用层**：用户开发的应用程序，基于 LVGL 提供的 API 构建界面和功能。

### 1.1==硬件抽象层 (HAL)==

硬件抽象层（HAL）是 LVGL 和底层硬件之间的桥梁，负责将硬件特性抽象成通用的接口。这些接口包括显示刷新、输入设备（如触摸屏、按钮等）处理以及系统时钟管理。`lv_hal` 文件夹，这里是所有硬件抽象接口的核心，开发者通过实现这些接口，将 LVGL 与底层硬件驱动结合

`lv_hal_disp.h`：显示接口的抽象 

`lv_hal_indev.h`：输入设备接口的抽象

`lv_hal_tick.h`：时钟和定时功能的接口



在硬件平台上，需要开发者实现显示器的刷屏函数、输入设备的读写函数等。LVGL 提供了抽象接口，开发者只需实现对应的回调函数，将硬件与 LVGL 结合起来。

```c
// 显示刷新回调函数例子
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    // 将显示缓冲区中的数据写入到显示屏
    display_write(area->x1, area->y1, area->x2, area->y2, color_p);

    // 刷新结束，通知 LVGL
    lv_disp_flush_ready(disp_drv);
}

```

```c
// 输入设备回调函数例子
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    // 获取触摸屏的坐标和状态
    get_touchpad_status(&data->point.x, &data->point.y, &data->state);
}

```

### 1.2 核心模块

核心模块负责界面的绘制、布局、事件处理、动画、主题等。它将图形界面中的各种控件抽象为对象（如按钮、文本框、进度条等），并提供丰富的事件系统，使开发者可以轻松响应用户交互。



`lv_core/lv_obj.h`：所有控件的基类，所有 UI 元素都继承自lv_obj

`lv_core/lv_style.h`：负责控件的样式管理

`lv_core/lv_event.h`：事件处理机制



### 1.3 应用层

应用层是开发者的编程领域，基于 LVGL 提供的 API，开发者可以构建自己的 GUI 应用。在应用层，开发者可以创建 UI 元素、设置它们的属性和事件处理函数，并通过 LVGL 提供的布局管理来组织界面布局。



## 2.与硬件交互的代码示例

以 ESP32 驱动 ST7796S 显示屏和 XPT2046 触摸屏为例，展示如何结合硬件和 LVGL 进行交互。

### 2.1 驱动显示屏

在 ESP32 开发环境中，使用 LVGL 时需要将显示屏的驱动与 LVGL 的显示刷新机制结合。以 SPI 驱动的 ST7796S 为例，通常通过 `lv_disp_drv_t` 结构体实现显示刷新回调。

```c
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    st7796s_send_pixels(area->x1, area->y1, area->x2, area->y2, (uint16_t *)color_p);
    lv_disp_flush_ready(disp_drv);  // 刷新完成后通知 LVGL
}

```

### 2.2 驱动触摸屏

类似地，触摸屏的输入数据通过 `lv_indev_drv_t` 结构体与 LVGL 交互。

```c
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    int touch_x, touch_y;
    bool is_pressed = xpt2046_read(&touch_x, &touch_y);  // 从硬件读取触摸状态和坐标
    if (is_pressed) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touch_x;
        data->point.y = touch_y;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

```



## 3.HAL层详细

在 LVGL 中，HAL 主要包含以下几个模块：

**显示驱动（Display Driver）**：负责与显示设备的交互，将 LVGL 渲染的图像传递给显示屏。

**输入设备驱动（Input Device Driver）**：负责从触摸屏、按键等设备获取输入数据。

**定时器（Tick Interface）**：提供系统时钟和定时服务，用于动画、任务管理等。

每个模块都有对应的抽象接口和回调函数，这些接口定义了硬件和 LVGL 之间的交互方式。开发者需要根据特定的硬件平台，编写具体的实现。

### 1.显示驱动 (Display Driver)

显示驱动是 LVGL HAL 的核心部分，负责将 LVGL 渲染的图形数据发送到物理显示设备。典型的显示器驱动器流程如下：

1. **LVGL 渲染**：LVGL 内部会先将控件绘制到显示缓冲区中（通常是内存中的一个缓冲区）。
2. **刷新显示器**：通过注册的显示驱动回调函数，将这个缓冲区中的数据发送到实际的显示设备中。
3. **显示完成通知**：显示器完成数据刷新后，通知 LVGL 刷新过程已经完成，允许进行下一次渲染。

#### 1.1 显示驱动的关键结构和函数

LVGL 中显示驱动的关键数据结构为 `lv_disp_drv_t`，这个结构体定义了显示器驱动的回调函数，以及显示设备的一些配置参数。

**关键结构体**：

```c
typedef struct {
    void (*flush_cb)(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
    void (*rounder_cb)(struct _lv_disp_drv_t * disp_drv, lv_area_t * area);
    void (*set_px_cb)(struct _lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t opa);
    // 其他配置项...
} lv_disp_drv_t;
```

- **`flush_cb`**：刷新回调函数，负责将缓冲区内容刷新到显示屏。
- **`rounder_cb`**：用于调整渲染区域的大小（可选）。
- **`set_px_cb`**：自定义像素设置函数（可选）。

**实现刷新回调函数**： 刷新回调函数负责将 LVGL 的显示缓冲区内容发送到物理屏幕上。以 SPI 驱动的 LCD 为例，`flush_cb` 会通过 SPI 接口将缓冲区中的数据写入显示屏。

```c
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    st7796s_send_pixels(area->x1, area->y1, area->x2, area->y2, (uint16_t *)color_p);  // 将像素数据发送到显示屏
    lv_disp_flush_ready(disp_drv);  // 通知 LVGL 刷新完成
}

```

**分配显示缓冲区**： LVGL 需要一个显示缓冲区来存储绘制的图像数据。在嵌入式系统中，缓冲区通常存储在内存中，并通过 DMA 或直接写入的方式发送到屏幕。

```c
static lv_color_t buf1[LV_HOR_RES_MAX * 10];  // 定义显示缓冲区
static lv_disp_draw_buf_t disp_buf;
lv_disp_draw_buf_init(&disp_buf, buf1, NULL, LV_HOR_RES_MAX * 10);  // 初始化显示缓冲区

```

### 2.输入设备驱动 (Input Device Driver)

输入设备驱动负责处理用户的输入动作，例如触摸屏、按键、鼠标等。LVGL 提供了通用的输入设备抽象，开发者可以实现具体硬件平台的输入驱动。

#### 2.1 输入设备的关键结构和函数

输入设备驱动使用 `lv_indev_drv_t` 结构体来配置驱动参数。它包含输入设备的类型、读取输入的回调函数等。

**关键结构体**：

```c
typedef struct {
    lv_indev_type_t type;        // 输入设备类型（如触摸屏、键盘、鼠标等）
    void (*read_cb)(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
    // 其他配置项...
} lv_indev_drv_t;

```

**`read_cb`**：输入读取回调函数，负责读取输入设备的状态（例如触摸屏的坐标或按键的状态）。



**实现读取回调函数**： 读取回调函数负责从输入设备（如触摸屏）获取输入状态（例如坐标和按压状态）。根据硬件接口不同，可能使用 I2C、SPI 或其他通信方式读取输入。

```c
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    int touch_x, touch_y;
    bool is_pressed = xpt2046_read(&touch_x, &touch_y);  // 从硬件读取触摸状态和坐标
    if (is_pressed) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touch_x;
        data->point.y = touch_y;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

```

### 3.Tick（定时器）管理

LVGL 依赖系统的时钟（Tick）来进行动画、任务处理等定时操作。因此，开发者需要为 LVGL 提供一个系统时钟的实现。LVGL 中的定时器（tick）模块非常重要，用于时间相关的任务，例如刷新屏幕、处理输入、执行动画等。

在嵌入式系统中，通常会有一个硬件定时器或系统滴答定时器，开发者可以利用这些资源为 LVGL 提供 Tick 计数。典型的实现方式是每隔 1 毫秒调用一次 `lv_tick_inc`。****

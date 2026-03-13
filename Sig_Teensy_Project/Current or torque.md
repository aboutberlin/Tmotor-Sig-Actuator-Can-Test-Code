接下来，我们需要确定这里的扭矩和电流到底是什么关系。这个要搞清楚。根据原本的代码写的，void receive_motor_feedback() {
  #if MOTOR_BRAND == 0
      // ---- SIG 路径：按 msgR.id 解析 pos/iq 等 ----
      CAN_message_t msgR;
      while (Can3.read(msgR)) {
          switch (msgR.id) {
              case ID_M1_POSVEL:
                  sig_m1.unpack_pos_vel(msgR, initial_pos_1);
                  break;
              case ID_M1_IQ: {
                  float iq = *(float *)&msgR.buf[4];
                  sig_m1.torque = iq * KT_1;
                  break;
              }
              case ID_M2_POSVEL:
                  sig_m2.unpack_pos_vel(msgR, initial_pos_2);
                  break;
              case ID_M2_IQ: {
                  float iq = *(float *)&msgR.buf[4];
                  sig_m2.torque = iq * KT_2;
                  break;
              }
              default:
                  // 如果有其它 SIG 消息也可能在这里处理
                  break;
          }
      }
      // SIG：把实测扭矩同步到全局测量变量（如果你还同时用sig_m?.torque也可以）
      M1_torque_meas = sig_m1.torque;
      M2_torque_meas = sig_m2.torque;
  #define KT_1 0.67f
#define KT_2 0.67f，其实本质就是读取到了电流之后*这个数值，一般性iq和id是什么关系？我这个是减速电机。至于电流发送，看起来是发送的是torque指令，然后    #if MOTOR_BRAND == 0
      for (int i = 0; i < 4; ++i) receive_motor_feedback();
      sig_m1.sig_torque_cmd(M1_torque_command/9.76);
      sig_m2.sig_torque_cmd(M2_torque_command/9.76);。这样，做了什么处理？可以看看手册确定一下？






      Iq 和扭矩关系本质是：T ≈ Kt * Iq，减速器后还会乘 gear_ratio（和效率）。



      扭矩命令走 0x0E Set_Input_Torque，写入 float32 Nm
      读取是，0x14 Get_Iq（A）：Motor_Control_Sig.cpp:193
      0x1C Get_Torques（Nm）：Motor_Control_Sig.cpp:199

      计算关系是，Iq_measured * torque_constant * gear_ratio = Set_Input_Torque


      原始代码读取是，sig_m?.torque = iq * KT_?，这是把 Iq(A) 乘一个等效系数得到扭矩估计（Nm）。是0.67
      那么写入sig_m1.sig_torque_cmd(M1_torque_command/9.76);




      Id: 磁链轴电流（通常设 0，除弱磁工况）。
      Iq: 转矩轴电流（主要决定扭矩）。


      转子机械侧（motor rotor side），Iq、torque_constant、input_torque 都在这侧
      关系：T_rotor ≈ Kt × Iq
    你给的 Kt=0.097 Nm/A 就是这个关系系数


输出轴侧（过减速器后）


看起来设置完控制模式后，我们需要设置输入模式（input_mode），才能决定我们控制力矩的算法。下面会说明两种电机自带的力矩控制算法。 力矩控制的单位是 Nm（转子侧），而驱动器固件中电流单位是 A，所以还需要设置力矩常数，以让驱动器能够将 Nm 转换为电流，从而按需求驱动电机输出力矩。 Plain Text # 力矩常数大约等于 odrv0.axis0.motor.config.torque_constant = 8.23/12.3，转矩常数	电流转矩转换系数（转子侧）	0.09700 Nm/A	odrv0.axis0.motor.config.torque_constant、

接下来，我想和你说的是，sig电机这个东西数值很混乱，不统一




注意，力矩控制，
0x0E 明确是 Input_Torque，单位 Nm，
0x14 明确是 Iq_Measured，单位 A。
0x1C 明确是 Torque，单位 Nm
都是转子侧！而不是输出侧
因此我们只会在显示的地方显示输出的，代码内部都是转子侧
Iq_measured * torque_constant（input torque） * gear_ratio = 输出侧扭矩

这里的constant是odrv0.axis0.motor.config.torque_constant，是转子侧的

odrv0.axis0.motor.config.torque_constant理论上应该是0.0568？




好的，再给你一些内容。极对数	电机的磁极对数（poles / 2）	10	odrv0.axis0.motor.config.pole_pairs 减速比	电机输出轴与编码器之间的减速比	9.67	odrv0.axis0.motor.config.gear_ratio。电机速度	最大电机速度（RPM）	186.14 RPM（输出轴）	odrv0.axis0.controller.config.vel_limit（odrivetool 里的单位是转子侧 转/s）驱动温度保护	驱动器温度保护阈值	25 ~ 80°C	odrv0.axis0.motor.fet_thermistor.config.enabled = True odrv0.axis0.motor.fet_thermistor.config.temp_limit_lower odrv0.axis0.motor.fet_thermistor.config.temp_limit_upper 电机温度保护	电机温度保护阈值	25 ~ 90°C	odrv0.axis0.motor.motor_thermistor.config.enabled = True odrv0.axis0.motor.motor_thermistor.config.temp_limit_lower odrv0.axis0.motor.motor_thermistor.config.temp_limit_upper相间电阻	电机相电阻	0.215 Ω	odrv0.axis0.motor.config.phase_resistance 相间电感	电机相电感	0.000090 H	odrv0.axis0.motor.config.phase_inductance 转矩常数	电流转矩转换系数（转子侧）	0.09700 Nm/A	odrv0.axis0.motor.config.torque_constant转动惯量	电机转动部分的惯量（扭矩模式需要）	0.00	odrv0.axis0.controller.config.inertia控制带宽 电流环的带宽，越高越快但可能震荡	1000.0	odrv0.axis0.motor.config.current_control_bandwidth Q 轴限流	电机最大 Q 轴电流	60.0 A	odrv0.axis0.motor.config.current_lim 斜坡斜率	电流变化速率限制	1.000 N·m/s	odrv0.axis0.controller.config.torque_ramp_rate3.3.3 力矩控制 首先需要将控制模式更改为力矩模式 Python odrv0.axis0.controller.config.control_mode = 1 设置完控制模式后，我们需要设置输入模式（input_mode），才能决定我们控制力矩的算法。下面会说明两种电机自带的力矩控制算法。 力矩控制的单位是 Nm（转子侧），而驱动器固件中电流单位是 A，所以还需要设置力矩常数，以让驱动器能够将 Nm 转换为电流，从而按需求驱动电机输出力矩。 Plain Text # 力矩常数大约等于 odrv0.axis0.motor.config.torque_constant = 8.23/12.3 直接力矩控制（Toruqe Control） 这是最简单的力矩（电流）控制模式，电机会以全力到达目标力矩，电机可能会很容易触发报警，所以我们并不推荐使用，使能如下：。。。上面这些东西只是给你参考，因为据我所知，这个电机的手册有各种问题。我需要你给我先说一下，假设我就用直接电流控制，也就是直接力矩控制，有很多限制可以从odrive tool关闭，哪些是可以关闭的？其次，再给我说一下各个参数的意思，尤其是电极侧和电子侧


torque_ramp_rate 设很大（等效不限制斜率）
力矩模式速度限幅联动关闭（有的固件是 enable_torque_mode_vel_limit=False）
vel_limit 设高一点（避免低速限幅干预）
watchdog 关闭（若你确定通讯稳定）
不建议关闭（建议始终开着）：

FET 温度保护
电机温度保护
电流限制（current_lim）
总线电压相关保护（欠压/过压）
驱动错误机制本身
一句话：可以关“控制体验限制”，不要关“硬件安全保护”。

odrv0.axis0.motor.config.gear_ratio

odrv0.axis0.controller.config.vel_limit
odrv0.axis0.controller.config.enable_vel_limit
odrv0.axis0.controller.config.enable_torque_mode_vel_limit
odrv0.axis0.motor.fet_thermistor.config.enabled = True 
odrv0.axis0.motor.motor_thermistor.config.enabled = True 
odrv0.axis0.motor.config.current_lim
odrv0.axis0.controller.config.torque_ramp_rate
# 本地固件功能配置 590adad7

由 AI 辅助接入用户提供的官方自定义构建配置。

- 基线：ArduCopter 4.7.2，commit fb773567896014da007bca58358704d4a0d8516f。
- 板型：MicoAir405v2。
- 来源：copter-MicoAir405v2-590adad7.tar.gz 中的 extra_hwdef.dat。
- 完整原文件复制到 libraries/AP_HAL_ChibiOS/hwdef/MicoAir405v2/custom-590adad7.inc。
- hwdef.dat 末尾 include 此文件，覆盖顺序与官网构建的 extra-hwdef 一致。

在已安装 ArduPilot 编译工具链的 Linux/WSL 环境中，从源码根目录执行：

```sh
./waf configure --board MicoAir405v2
./waf copter
```

无需再次指定 --extra-hwdef。此配置针对 Copter，不用于其他机型固件。
输出路径为 build/MicoAir405v2/bin/arducopter.apj。

验证：直接从压缩包读取配置并逐字节核对；使用此版本官方 ChibiOSHWDef 解析器，对比原板级配置加原 extra_hwdef 与修改后板级配置，config、intdefines、alllines、env_vars 完全一致。
本次未编译、未仿真、未刷机，不宣称二进制与原固件逐字节一致。

本次恢复的是编译时功能选择，不包含飞控中保存的 PID、串口、EKF 数据源等运行参数；没有新增起飞或高度控制逻辑。
回退：去掉 hwdef.dat 末尾新增的注释和 include 行，即恢复原板级配置。

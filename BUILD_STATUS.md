# 交付整理验证状态

本仓库为第二阶段整合工程，公开交付地址为https://github.com/heigangrusy-dot/sp_stage2，主分支为main。第一阶段单独保留在sptest。

- chassis/：底盘完整工程，已包含原工作区4个未提交文件的最终修改；Debug编译、链接通过。
- gimbal/：取自同学GitHub主分支862a5ea。控制代码和历史完整保留，但仓库没有上传所需的sp_middleware。已按用户指定改用D:/Study/SP/sp_stage2/sp_middleware。该目录的CAN文件与旧下载目录完全相同，仍不包含最新版云台所需的接口。
- 云台Debug构建失败：sp::CAN缺少service()与rx_len。需从同学实际最终工程补齐对应CAN中间件；不应删掉调用来掩盖缺失依赖。
- 两个原工程均未修改。两个工程分别构建、分别烧录到各自C板；本次未烧录或进行实车测试。
- 根README已完成，按当前参数和实际启用的串口通道编写；云台依赖与构建问题如上，供后续接手补齐。

## 历史

双方原始提交作为合并祖先保留，提交ID、作者、日期保持不变。history/chassis与history/gimbal分支指向导入时的原分支头。具体来源与提交号见SOURCE_HISTORY.json。

## 构建

分别进入chassis或gimbal目录，在配置好STM32CubeCLT工具链的环境执行：

```powershell
cmake --preset Debug
cmake --build build/Debug
```

编译输出在各工程build/中，已忽略，不提交。补齐云台最终依赖后需要重新构建验证。

# 📋 RViz 测试脚本使用指南

本目录包含了多个RViz测试脚本，用于验证和演示RViz功能。

## 🧪 测试脚本总览

### 1. **quick_test_rviz.py** - 快速测试 ⚡
**用途**: 快速验证RViz基本功能和配置文件
**时间**: ~30秒
**适用**: 日常检查、CI/CD

```bash
python3 quick_test_rviz.py
```

**检查内容**:
- ✅ RViz配置文件存在性
- ✅ ROS包安装状态  
- ✅ 基础话题可用性
- ✅ RViz启动能力

---

### 2. **test_rviz.py** - 全面测试 🔍
**用途**: 完整的RViz系统测试
**时间**: ~5分钟
**适用**: 系统验证、故障诊断

```bash
python3 test_rviz.py
```

**测试内容**:
- 📁 配置文件完整性检查
- 📦 ROS包依赖验证
- 🚀 仿真环境测试
- 🎯 各RViz配置启动测试
- 🗺️ SLAM集成测试
- 🤖 AI SLAM集成测试
- 📊 系统性能测试
- 📄 详细测试报告生成

**输出**: 
- 终端彩色报告
- `rviz_test_report.txt` 文件

---

### 3. **demo_rviz.py** - 交互式演示 🎮
**用途**: 引导式RViz功能演示
**时间**: ~10-15分钟
**适用**: 学习、培训、展示

```bash
python3 demo_rviz.py
```

**演示内容**:
1. **基本机器人视图** - 3D模型、传感器数据
2. **SLAM建图视图** - 实时建图过程
3. **AI SLAM对比** - 传统vs AI增强SLAM
4. **导航功能** - 路径规划、目标设置

**特色**:
- 🎯 逐步引导
- 📋 详细说明
- 🎮 交互式操作
- 💡 实用技巧

---

### 4. **test_single_rviz.sh** - 单配置测试 🎯
**用途**: 测试特定RViz配置
**时间**: ~1分钟
**适用**: 配置调试、快速验证

```bash
# 查看可用配置
./test_single_rviz.sh --list

# 测试特定配置
./test_single_rviz.sh robot_view
./test_single_rviz.sh slam_view
./test_single_rviz.sh navigation_view
./test_single_rviz.sh full_navigation
```

**功能**:
- 🔍 单配置文件验证
- 🚀 自动环境启动
- 🎯 直接RViz测试
- 🧹 自动清理

---

## 🚀 快速使用指南

### 日常检查 (30秒)
```bash
python3 quick_test_rviz.py
```

### 深度诊断 (5分钟)
```bash
python3 test_rviz.py
```

### 学习演示 (15分钟)
```bash
python3 demo_rviz.py
```

### 配置调试
```bash
./test_single_rviz.sh <config_name>
```

## 📊 测试结果解读

### ✅ 全部通过
- 系统完全正常
- 所有功能可用
- 可以正常使用

### ⚠️ 部分失败 (>80%通过)
- 基本功能正常
- 可能有轻微配置问题
- 建议检查失败项

### ❌ 多项失败 (<80%通过)
- 系统配置问题
- 需要检查依赖安装
- 建议重新构建工作空间

## 🔧 常见问题解决

### RViz启动失败
```bash
# 检查显示环境
export DISPLAY=:0

# 检查X11转发 (SSH)
ssh -X username@hostname
```

### 话题数据缺失
```bash
# 检查仿真是否正常
ros2 topic list
ros2 topic echo /scan --once

# 重启仿真环境
pkill -f gzserver && pkill -f gzclient
```

### 性能问题
```bash
# 降低仿真质量
ros2 launch my_robot_simulation combined.launch.py gui:=false

# 关闭不必要的显示项
# 在RViz中取消勾选大数据量的显示项
```

## 🎯 最佳实践

1. **新环境验证**: 运行`quick_test_rviz.py`
2. **问题诊断**: 运行`test_rviz.py`查看详细报告
3. **功能学习**: 运行`demo_rviz.py`进行交互式学习
4. **配置调试**: 使用`test_single_rviz.sh`测试具体配置

## 📚 相关文档

- **README.md**: RViz使用指南主文档
- **src/my_robot_description/rviz/**: RViz配置文件
- **rviz_test_report.txt**: 详细测试报告 (运行test_rviz.py后生成)

---

💡 **提示**: 建议在使用RViz功能前先运行快速测试，确保系统环境正常。
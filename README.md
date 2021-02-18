# TaiChi

## 循环

情况一：刚完整经过普通点，下一个点为普通点或携带点

+ 若正对下一点，沿线直行，到前端传感器接触下一条线继续

+ 若背对下一点，沿线后退，到后端传感器接触下一条线继续

+ 继续直行或后退或转向

+ 继续循环

情况二：刚完整经过普通点，下一个点为抓取点

+ 若爪子上无环且状态不正常，停止前进，再次检测爪子是否正常

+ 若爪子上无环且状态正常，沿线直行，在抓取位置停止，进行抓取，抓取过程中若出现失败，则检测爪子是否正常；若成功，则视为爪子有环

+ 继续沿线直行，到前端传感器接触下一条线继续

+ 继续直行或后退或转向

+ 将越过的点视为普通点

+ 继续循环

情况三：刚完整经过普通点，下一个点为释放点（使用机械臂）

+ 沿线直行，在释放位置停止

+ 若爪子有环，进行释放，之后视为爪子无环

+ 下一点朝向为后

+ 继续循环

情况四：刚完整经过释放点（使用机械臂），下一个点为普通点

+ 沿线后退，到后端传感器接触下一条线停止

+ 机械臂复原

+ 继续后退或转向

+ 继续循环

情况五：刚完整经过普通点，下一个点为释放点（从底盘）

+ 沿线直行，到后端传感器接触下一条线停止

+ 下一点朝向为后，继续循环

情况六：刚完整经过释放点（从底盘），下一个点为普通点

+ 沿线后退，到前端传感器接触线继续

+ 继续沿线后退，到后端传感器接触线继续

+ 继续后退或转向

+ 继续循环
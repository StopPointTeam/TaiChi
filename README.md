# TaiChi

## 循环

情况一：刚完整经过普通点，下一个点为普通点

+ 若正对下一点，沿线直行，到前端传感器接触下一条线为止

+ 若背对下一点，沿线后退，到后端传感器接触下一条线为止

+ 继续直行或后退或转向

+ 继续循环

情况二：刚完整经过普通点，下一个点为抓取点

+ 沿线直行，在抓取位置停止

+ 抓取

+ 继续沿线直行，到前端传感器接触下一条线为止

+ 继续直行或后退或转向

+ 抓取完成后，将越过的点视为普通点，继续循环

情况三：刚完整经过普通点，下一个点为释放点

+ 沿线直行，在释放位置停止

+ 释放

+ 释放完成后，下一点朝向为后，继续循环

情况四：刚完整经过释放点，下一个点为普通点

+ 沿线后退，到后端传感器接触下一条线为止

+ 机械臂复原

+ 继续后退或转向

+ 继续循环
<center><b><font size = 8>中山大学计算机学院<br>计算机图形学<br>期末作业报告</font></b> </center>

| 课程         | 小组成员 |
| ------------ | -------- |
| 计算机图形学 |          |

## 一、更多的光源类型

### 面光源

面光源采用矩形加散射光材质

### 聚光灯

聚光是位于环境中某个位置的光源，它只朝一个特定方向照射光线，只有在聚光方向的特定半径内的物体才会被照亮。我们采用聚光方向`light_direction`，切光角`cutOff`，外切光角`outerCutOff`来表示一个聚光`spotlight`

- `light_direction`：聚光灯发射光线的方向
- `cutOff`：指定了聚光半径的切光角，落在这个角度之外的物体将会受到衰减系数的影响
- `outerCutOff`：指定了聚光半径的外切角，在这个角度之外的物体将不会被这个聚光灯所照亮。

将射线与相反的聚光灯方向做点积，得到的就是他们两者的角度的余弦值。为了平滑边缘，再定义一个外圆锥，定义衰减系数方程使得在内圆锥内的衰减系数为1.0，而在内外圆锥之间就将衰减到[0,1]，而在外圆锥之外将会是0:
$$
I=\frac{ \theta - \gamma}{\epsilon}
$$

```C++
virtual color emitted(const ray& r_in, const hit_record& rec, double u, double v,
    const point3& p) const override {
    float theta = dot(unit_vector(r_in.direction()), -light_direction);
    float epsilon = cutOff - outerCutOff;
    float intensity = clamp((theta - outerCutOff) / epsilon, 0.0, 1.0);
    intensity = pow(intensity, 1.8);
    return intensity * emit->value(u, v, p);
}
```



## 二、为场景物体添加纹理

## 三、支持网格模型加载

## 四、使用蒙特卡洛积分方法求解渲染方程



## 五、支持多线程加速渲染计算过程

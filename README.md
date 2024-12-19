# `<center>` 中山大学计算机学院

# `<center>` 计算机图形学

# `<center>` 期末作业报告

<center> 课程名称：Computer graphics </center>

## 一、项目内容介绍

我们小组完成的任务是光线追踪渲染器完善的任务。跟着Assignment4的任务，我们已经实现了简单的光纤追踪器的功能，其中包括了超采样、景深、球类和立方体的类实现、运动模糊、BVH树等诸多内容。我们按照任务要求，主要从以下五个方面对这个基本的光线追踪渲染器进行了完善

1. 更多的光源类型：实现了面光源、聚光灯、
2. 为场景中的物体添加了更多更丰富的纹理
3. 编写了模型载入的函数，其中包括了模型如何载入以及与载入模型的相交判断等函数
4. 使用了蒙特卡洛方法对路径追踪的过程进行了完善
5. 使用CPU的多线程并行来进行画面渲染

以下是每个部分的具体介绍。

## 二、更多的光源类型

### 面光源

面光源采用矩形加散射光材质

### 聚光灯

聚光是位于环境中某个位置的光源，它只朝一个特定方向照射光线，只有在聚光方向的特定半径内的物体才会被照亮。我们采用聚光方向 `light_direction`，切光角 `cutOff`，外切光角 `outerCutOff`来表示一个聚光 `spotlight`

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

## 三、为场景物体添加纹理

首先使用stb_image库中的函数从图像中获取像素数据及其坐标，在`hit_record`结构体中加上纹理坐标变量u、v，使得计算碰撞点时还得到纹理坐标。 同时在sphere类中加上计算纹理坐标的`get_sphere_uv`函数，并在计算碰撞的`hit`函数例调用它。

首先计算经度：
$$
\phi = \arctan2(p.z(), p.x())
$$
	然后计算纬度：
$$
\theta = \arcsin(p.y())
$$
	再计算U坐标：
$$
u = 1 - \frac{\phi + \pi}{2\pi}
$$
	计算V坐标：
$$
v = \frac{\theta + \frac{\pi}{2}}{\pi}
$$
	最后得到将纹理映射到球体的函数。

```
void get_sphere_uv(const vec3& p, double& u, double& v) {
    auto phi = atan2(p.z(), p.x());
    auto theta = asin(p.y());
    u = 1 - (phi + pi) / (2 * pi);
    v = (theta + pi / 2) / pi;
}
```

## 四、支持网格模型加载

此项目通过创建三角形片元渲染类triangle.h以实现对obj网格模型文件的加载，并编写obj_input类实现对.obj文件中的点面数据的读取。

#### 1. 网格模型文件(.obj)读取的实现

读取有以下接口：

`filename`：要加载的 `.obj` 文件的路径。

`vertices`：三维空间中的点集。

`normals`：法向量。

`faces`：包含构成该面的顶点索引。

`materialFaces`：面所对应的颜色或材质属性。

```
static bool loadOBJ(
        const std::string& filename, 
        std::vector<point3>& vertices, 
        std::vector<vec3>& normals, 
        std::vector<vec3>& texCoords, 
        std::vector<std::vector<int>>& faces,
        std::map<std::vector<int>, vec3>& materialFaces)
```

​	

将文件打开后遍历一遍，根据类型将所有顶点信息、法向量信息及面信息压入vector中。

	std::string line;
	vec3 currentColor;
	while (std::getline(file, line)) {
	    std::istringstream iss(line);
	    std::string type;
	    iss >> type;
	
	    if (type == "v") {
	        point3 vertex;
	        iss >> vertex[0] >> vertex[1] >> vertex[2];
	        vertices.push_back(vertex);
	    }
	    else if (type == "vn") {
	        vec3 normal;
	        iss >> normal[0] >> normal[1] >> normal[2];
	        normals.push_back(normal);
	    }
	    else if (type == "vt") {
	        vec3 texCoord;
	        iss >> texCoord[0] >> texCoord[1];
	        texCoords.push_back(texCoord);
	    }
	    else if (type == "f") {
	        std::vector<int> face;
	        std::string vertexStr;
	        while (iss >> vertexStr) {
	            std::istringstream vss(vertexStr);
	            std::string vertexIndexStr;
	            std::getline(vss, vertexIndexStr, '/');
	            int vertexIndex = std::stoi(vertexIndexStr) - 1; // Convert 1-based indices to 0-based
	            face.push_back(vertexIndex);
	        }
	        faces.push_back(face);
	        //auto p = std::make_pair(face, currentColor);
	        //materialFaces.insert(std::make_pair(face, currentColor));
	        materialFaces[face] = currentColor;



#### 2.triangle类

##### 成员变量：

- `vec3 vertex0`：三角形的第一个顶点。

- `vec3 vertex1`：三角形的第二个顶点。

- `vec3 vertex2`：三角形的第三个顶点。

- `shared_ptr<material> mat_ptr`：表示三角形的材料属性的智能指针。

##### 构造函数：

- `triangle(const vec3& v0, const vec3& v1, const vec3& v2, shared_ptr<material> mat)`：构造函数，初始化三角形的三个顶点和材料属性。

##### 成员函数：

`virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override`：用于检测射线是否与三角形相交。重写了 `hittable` 类中的 `hit` 方法。

`virtual bool bounding_box(double t0, double t1, aabb& box) const override`：用于计算三角形的轴对齐边界框。

```

class triangle : public hittable {
public:
    triangle(const vec3& v0, const vec3& v1, const vec3& v2, shared_ptr<material> mat)
        : vertex0(v0), vertex1(v1), vertex2(v2), mat_ptr(mat) {}

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override {
        vec3 edge1 = vertex1 - vertex0;
        vec3 edge2 = vertex2 - vertex0;
        vec3 h = cross(r.direction(), edge2);
        double a = dot(edge1, h);
        if (a > -0.00001 && a < 0.00001)
            return false;
        double f = 1.0 / a;
        vec3 s = r.origin() - vertex0;
        double u = f * dot(s, h);
        if (u < 0.0 || u > 1.0)
            return false;
        vec3 q = cross(s, edge1);
        double v = f * dot(r.direction(), q);
        if (v < 0.0 || u + v > 1.0)
            return false;
        double t = f * dot(edge2, q);
        if (t < t_min || t > t_max)
            return false;
        rec.t = t;
        rec.p = r.at(t);
        rec.mat_ptr = mat_ptr;
        vec3 normal = face_normal(vertex0, vertex1, vertex2);
        rec.set_face_normal(r, normal);
        return true;
    }

    virtual bool bounding_box(double t0, double t1, aabb& box) const override {
        vec3 min = minxyz(minxyz(vertex0, vertex1), vertex2);
        vec3 max = maxxyz(maxxyz(vertex0, vertex1), vertex2);
        box = aabb(min, max);
        return true;
    }

private:
    vec3 vertex0;
    vec3 vertex1;
    vec3 vertex2;
    shared_ptr<material> mat_ptr;
};
```



`load_obj_model` 的函数，它用于从 `.obj` 文件中加载3D模型，并将其转换为一个由多个 `triangle` 对象组成的列表。添加点进入三角形的同时，还加入了`rotation`旋转矩阵与`move`平移矩阵对全体点进行旋转移动以达到对模型进行放缩、旋转及移动的效果。

```
static hittable_list load_obj_model(const std::string& filename, std::shared_ptr<material> mat, hittable_list& world, std::vector<std::vector<double>> matrix, vec3 move) {
    std::vector<point3> vertices;
    std::vector<vec3> normals;
    std::vector<vec3> texCoords;
    std::vector<std::vector<int>> faces;
    std::map<std::vector<int>, vec3> materialFaces;

    OBJINPUT::loadOBJ(filename, vertices, normals, texCoords, faces, materialFaces);

    //std::cout << "test" << std::endl;

    for (int i = 0; i < vertices.size(); ++i) {
        rotation(vertices[i], matrix);
        vertices[i] += move;
    }

    //hittable_list obj_triangles;
    for (const auto& face : faces) {
        for (size_t i = 0; i < face.size() - 2; ++i) {
            int v0_idx = face[0], v1_idx = face[i + 1], v2_idx = face[i + 2];
            //rotation(vertices[v0_idx], matrix);
            //rotation(vertices[v1_idx], matrix);
            //rotation(vertices[v2_idx], matrix);
            vec3 tempColor = materialFaces[face];
            world.add(make_shared<triangle>(
                vertices[v0_idx], vertices[v1_idx], vertices[v2_idx], make_shared<lambertian>(tempColor)));
        }
    }

    return world;
}
```



## 五、使用蒙特卡洛积分方法求解渲染方程

### 5.1 使用重要性采样（Importance Sampling）技术来减少渲染图像中的噪声

* 修改了 `material`基类，添加了支持重要性采样的函数 `scatter`和 `scattering_pdf`。
* 对Lambertian材质进行了修改，实现了基于余弦分布的重要性采样，并计算了相应的PDF值。
* 修改了 `ray_color`函数，以考虑PDF值在光线追踪过程中的影响。

**蒙特卡洛积分**：通过采样来近似计算积分值。公式为：

$$
\int f(x) \, dx \approx \frac{f(r)}{p(r)}
$$

其中，$f(x)$ 是要积分的函数，$r$ 是根据概率密度函数$p(r)$采样得到的点。

**PDF的计算**：

- 对于Lambertian材质，使用余弦分布作为PDF，公式为：

  $$
  p(\text{direction}) = \frac{\cos(\theta)}{\pi}
  $$

  其中，$\theta$ 是光线方向与表面法线的夹角。

**对于随机半球采样，PDF为常数，公式为：**

$$
p(\text{direction}) = \frac{1}{2\pi}
$$

### 5.2 正交归一化基（ONB）

定义了一个ONB类，包含了构建ONB的方法（`build_from_w`）和将向量从标准坐标系转换到ONB坐标系的方法（`local`）。并且使用ONB类来改写Lambertian材质的散射函数，使其能够生成相对于表面法向量的随机散射方向，这里使用了蒙特卡洛方法中的随机采样技术来模拟光的散射。

在计算散射方向的概率密度时，使用了公式 `pdf = dot(uvw.w(), scattered.direction()) / pi;`。这里，`dot(uvw.w(), scattered.direction())`计算了散射方向与法向量之间的余弦值，除以π是为了将概率密度归一化到单位半球上。

### 5.3 对光源采样

通过直接对光源进行采样来改进渲染效果。均匀采样方向会导致在不重要的方向上浪费采样，因此考虑直接对光源进行采样。具体步骤为：

1. **计算PDF**：对于光源上的一个小区域 `dA`，其在单位球面上对应的方向区域 `dw`的概率密度函数 `p(direction)`被计算为 `distance^2(p,q) / (cos(alpha) * A)`，其中 `distance(p,q)`是采样点到光源点的距离，`alpha`是光线与光源表面法线的夹角，`A`是光源的面积。
2. **实现方法**：在 `ray_color`函数中硬编码了对光源的采样，并使用了上述PDF来计算最终的颜色值。
3. **改进**：为了解决光源两面发光导致的噪点问题，通过修改材质的 `emitted`函数和翻转光源的法线方向，使光源只向一个方向发光。

**相关公式**：

$$
\int f(x) \, dx \approx \frac{1}{N} \sum_{i=1}^{N} f(x_i)
$$

其中，`f(x)`是被积函数，`N`是采样点数，`x_i`是随机采样点。


## 六、支持多线程加速渲染计算过程

这一部分难度不大。因为我们的渲染本来是一行一行进行的，所以只需要先读取硬件有几个CPU核可以用于渲染，然后根据核数量将任务按行平均分给每一个线程，即可实现并行。这里需要注意的是，在分配行给线程的时候，要确保每一行都有被分配到一个线程，方法是 `int rows_per_thread = (gHeight + num_threads - 1) / num_threads;`，通过 `+num_threads - 1`，从而进行一种“向上取整到最接近的整数倍”的计算，就可以保证每行都被取到。

**具体代码：**

```
// 渲染函数，现在接受图像的一部分作为参数
void render_strip(int start_row, int end_row,const camera &cam, color background, const hittable& world) {
	for (int j = start_row; j < end_row; j++) {
		for (int i = 0; i < gWidth; i++) {
			color pixel_color(0, 0, 0);
			for (int s = 0; s < samples_per_pixel; ++s) {
				auto u = (i + random_double()) / (gWidth - 1);
				auto v = (j + random_double()) / (gHeight - 1);
				ray r = cam.get_ray(u, v);
				pixel_color += ray_color(r, background, world, max_depth);
			}
			write_color(i, j, pixel_color);
		}
	}
}
// 计算每个线程要渲染的行数
int num_threads = std::thread::hardware_concurrency(); // 获取可用的CPU核心数
int rows_per_thread = (gHeight + num_threads - 1) / num_threads;

// 创建线程向量
std::vector<std::thread> threads;

// 启动线程
for (int t = 0; t < num_threads; ++t) {
	int start_row = t * rows_per_thread;
	int end_row = std::min(start_row + rows_per_thread, gHeight);
	threads.emplace_back(render_strip, start_row, end_row,cam,background,world);
}

// 等待所有线程完成
	for (auto& th : threads) {
		th.join();
	}

```

## 七、最终效果

## 八、分工情况

吴琦：完成了多线程并行加速计算过程，并且参与实现蒙特卡洛积分方法求解渲染方程部分的代码，并且解决了《the rest of your life》教程中代码存在的诸多bug，使得程序正确运行。

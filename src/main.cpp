/*The MIT License (MIT)

Copyright (c) 2021-Present, Wencong Yang (yangwc3@mail2.sysu.edu.cn).

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.*/

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <array>
#include <vector>
#include <thread>
#include <iostream>
#include <memory>
#include<mutex>


#include "WindowsApp.h"
#include "vec3.h"
#include "ray.h"
#include "camera.h"
#include "hittable_list.h"
#include "sphere.h"
#include "material.h"
#include "moving_sphere.h"
#include "texture.h"
#include "xy_rect.h"
#include "box.h"
#include "constant_medium.h"
#include "bvh_node.h"
#include "triangle.h"
#include"cylinder.h"
#include"pdf.h"
//#include "obj_mesh.h"

static std::vector<std::vector<color>> gCanvas;		//Canvas

// The width and height of the screen
const auto aspect_ratio = 1.0;
const int gWidth = 640;
const int gHeight = static_cast<int>(gWidth / aspect_ratio);
const int max_depth = 64;
const int samples_per_pixel = 2048;

void rendering();

extern vec3 random_cosine_direction();



int main(int argc, char* args[])
{

	// Create window app handle
	WindowsApp::ptr winApp = WindowsApp::getInstance(gWidth, gHeight, "CGAssignment4: Ray Tracing");
	if (winApp == nullptr)
	{
		std::cerr << "Error: failed to create a window handler" << std::endl;
		return -1;
	}

	// Memory allocation for canvas
	gCanvas.resize(gHeight, std::vector<color>(gWidth));

	// Launch the rendering thread
	// Note: we run the rendering task in another thread to avoid GUI blocking
	std::thread renderingThread(rendering);

	// Window app loop
	while (!winApp->shouldWindowClose())
	{
		// Process event
		winApp->processEvent();

		// Display to the screen
		winApp->updateScreenSurface(gCanvas);

	}

	renderingThread.join();

	return 0;
}

void write_color(int x, int y, color pixel_color)
{
	auto r = pixel_color.x();
	auto g = pixel_color.y();
	auto b = pixel_color.z();
	// Out-of-range detection
	if (x < 0 || x >= gWidth)
	{
		std::cerr << "Warnning: try to write the pixel out of range: (x,y) -> (" << x << "," << y << ")" << std::endl;
		return;
	}

	if (y < 0 || y >= gHeight)
	{
		std::cerr << "Warnning: try to write the pixel out of range: (x,y) -> (" << x << "," << y << ")" << std::endl;
		return;
	}

	// Note: x -> the column number, y -> the row number

	// Replace NaN components with zero. See explanation in Ray Tracing: The Rest of Your Life.
	if (r != r) r = 0.0;
	if (g != g) g = 0.0;
	if (b != b) b = 0.0;

	auto scale = 1.0 / samples_per_pixel;
	r = sqrt(scale * pixel_color.e[0]);
	g = sqrt(scale * pixel_color.e[1]);
	b = sqrt(scale * pixel_color.e[2]);
	gCanvas[y][x] = color(clamp(r, 0.0, 0.999), clamp(g, 0.0, 0.999), clamp(b, 0.0, 0.999));

}

color ray_color(
	const ray& r, const color& background, const hittable& world,
	shared_ptr<hittable>& lights, int depth
) {
	hit_record rec;

	// If we've exceeded the ray bounce limit, no more light is gathered.
	if (depth <= 0)
		return color(0, 0, 0);

	// If the ray hits nothing, return the background color.
	if (!world.hit(r, 0.001, infinity, rec))
		return background;

	scatter_record srec;
	color emitted = rec.mat_ptr->emitted(r, rec, rec.u, rec.v, rec.p);
	if (!rec.mat_ptr->scatter(r, rec, srec))
		return emitted;

	if (srec.is_specular) {
		return srec.attenuation
			* ray_color(srec.specular_ray, background, world, lights, depth - 1);
	}

	auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
	mixture_pdf p(light_ptr, srec.pdf_ptr);

	ray scattered = ray(rec.p, p.generate(), r.time());
	auto pdf_val = p.value(scattered.direction());

	return emitted
		+ srec.attenuation * rec.mat_ptr->scattering_pdf(r, rec, scattered)
		* ray_color(scattered, background, world, lights, depth - 1) / pdf_val;
}

hittable_list simple_light() {
	hittable_list objects;

	/*std::vector<std::vector<double>> rotation = {
		{1.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, 1.0f},
		{0.0f, -1.0f, 0.0f}
	};*/

	std::vector<std::vector<double>> rotation = {
		{1.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, -1.0f},
		{0.0f, 1.0f, 0.0f}
	};

	vec3 move(0.0f, 20.0f, 0.0f);


	auto light = make_shared<diffuse_light>(make_shared<constant_texture>(vec3(4, 4, 4)));
	auto pertext = make_shared<noise_texture>(4);
	objects.add(make_shared<sphere>(vec3(0, -1000, 0), 1000, make_shared<lambertian>(vec3(0.1, 0.1, 0.1))));
	objects.add(make_shared<sphere>(vec3(-20, 10,-10), 10, light));

	load_obj_model("../assets/SwordMinecraft.obj", make_shared<dielectric>(1.5), objects, rotation, move);

	return objects;
}


hittable_list cornell_box() {
	hittable_list objects;

	std::vector<std::vector<double>> rotation = { {3.535, -3.535, 0.0f}, {0.0f, 0.0f, -5.0f}, {3.535f, 3.535f, 0.0f}};
	vec3 move(300.0f, 50.0f, 80.0f);

	load_obj_model("../assets/SwordMinecraft.obj", make_shared<lambertian>(vec3(0.5, 0.7, 1.0)), objects, rotation, move);

	int nx, ny, nn;
	auto tex_data = stbi_load("../assets/1.jpg", &nx, &ny, &nn, 0);
	auto emat1 = make_shared<lambertian>(make_shared<image_texture>(tex_data, nx, ny));

	tex_data = stbi_load("../assets/tmp.jpg", &nx, &ny, &nn, 0);
	auto emat2 = make_shared<lambertian>(make_shared<image_texture>(tex_data, nx, ny));

	tex_data = stbi_load("../assets/3.jpg", &nx, &ny, &nn, 0);
	auto emat3 = make_shared<lambertian>(make_shared<image_texture>(tex_data, nx, ny));

	tex_data = stbi_load("../assets/earthmap.jpg", &nx, &ny, &nn, 0);
	auto emat4 = make_shared<lambertian>(make_shared<image_texture>(tex_data, nx, ny));
	objects.add(make_shared<sphere>(vec3(355, 390, 330), 80, emat4));

	tex_data = stbi_load("../assets/R.jpg", &nx, &ny, &nn, 0);
	auto emat5 = make_shared<lambertian>(make_shared<image_texture>(tex_data, nx, ny));

	auto center1 = point3(350, 250, 200);
	auto center2 = center1 + vec3(0, -30, 0);
	auto moving_sphere_material = make_shared<lambertian>(vec3(1.0, 1.0, 0.5));
	objects.add(make_shared<moving_sphere>(center1, center2, 0, 1, 50, moving_sphere_material));

	auto red = make_shared<lambertian>(color(.65, .05, .05));
	auto white = make_shared<lambertian>(color(.73, .73, .73));
	auto green = make_shared<lambertian>(color(.12, .45, .15));
	auto skyblue = make_shared<lambertian>(color(.5, .7, 1.0));
	auto light = make_shared<diffuse_light>(make_shared<constant_texture>(vec3(0.9, 1.0, 0.8)));
	auto spotlight = make_shared<spot_light>(make_shared<constant_texture>(vec3(23, 25, 25)), vec3(0, -1, 0), 0.75, 0.15);
	auto mirror = make_shared<metal>(vec3(0.5, 0.92, 1.0), 0.001);


	objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, green));
	objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, red));
	//objects.add(make_shared<flip_face>(make_shared<xz_rect>(213, 343, 227, 332, 554, light)));
	// 球光源
	 objects.add(make_shared<sphere>(point3(500, 65, 300), 30, light));

	//聚光
	objects.add((make_shared<xz_rect>(213, 343, 227, 332, 554, spotlight)));
	//平行光源
	objects.add((make_shared<xy_rect>(-200, 800, -200, 800, -810, light)));

	objects.add(make_shared<xz_rect>(0, 555, 0, 555, 555, white));
	objects.add(make_shared<xz_rect>(0, 555, 0, 555, 0, mirror));
	objects.add(make_shared<xy_rect>(0, 555, 0, 555, 555, white));

	shared_ptr<material> aluminum = make_shared<lambertian>(color(0.8, 0.85, 0.88));
	shared_ptr<hittable> box1 = make_shared<box>(point3(0, 0, 0), point3(165, 330, 165), aluminum);
	box1 = make_shared<rotate_y>(box1, 15);
	box1 = make_shared<translate>(box1, vec3(265, 0, 295));
	objects.add(box1);

	shared_ptr<hittable> box2 = make_shared<box>(point3(0, 0, 0), point3(130, 130, 130), skyblue);
	box2 = make_shared<rotate_y>(box2, 30);
	box2 = make_shared<translate>(box2, vec3(50, 0, 370));
	objects.add(box2);

	shared_ptr<hittable> chest = make_shared<box>(point3(0, 0, 0), point3(100, 100, 100), emat1, emat2, emat3);
	chest = make_shared<rotate_y>(chest, -15);
	chest = make_shared<translate>(chest, vec3(100, 130, 350));
	objects.add(chest);

	auto glass = make_shared<dielectric>(1.5);
	objects.add(make_shared<sphere>(point3(100, 45, 100), 55, glass));

	//shared_ptr<hittable> chest = make_shared<box>(point3(0, 0, 0), point3(100, 100, 100), emat1);
	//chest = make_shared<rotate_y>(chest, -15);
	//chest = make_shared<translate>(chest, vec3(150, 0, 150));
	//objects.add(chest);

	return objects;
}

hittable_list cornell_smoke() {
	hittable_list objects;

	auto red = make_shared<lambertian>(make_shared<constant_texture>(vec3(0.65, 0.05, 0.05)));
	auto white = make_shared<lambertian>(make_shared<constant_texture>(vec3(0.73, 0.73, 0.73)));
	auto green = make_shared<lambertian>(make_shared<constant_texture>(vec3(0.12, 0.45, 0.15)));
	auto light = make_shared<diffuse_light>(make_shared<constant_texture>(vec3(4, 4, 4)));

	objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, green));
	objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, red));
	objects.add(make_shared<xz_rect>(113, 443, 127, 432, 554, light));
	objects.add(make_shared<xz_rect>(0, 555, 0, 555, 0, white));
	objects.add(make_shared<xy_rect>(0, 555, 0, 555, 555, white));
	objects.add(make_shared<xz_rect>(0, 555, 0, 555, 555, white));

	shared_ptr<material> aluminum = make_shared<metal>(color(0.8, 0.85, 0.88), 0.0);
	shared_ptr<hittable> box1 = make_shared<box>(point3(0, 0, 0), point3(165, 330, 165), aluminum);
	box1 = make_shared<rotate_y>(box1, 15);
	box1 = make_shared<translate>(box1, vec3(265, 0, 295));
	objects.add(box1);

	auto glass = make_shared<dielectric>(1.5);
	objects.add(make_shared<sphere>(point3(190, 90, 190), 90, glass));

	return objects;
}

hittable_list custom_scene() {
	hittable_list objects;

	// 地面：使用带有纹理的Lambertian材质
	auto even_texture = make_shared<constant_texture>(vec3(0.8, 0.8, 0.8));
	auto odd_texture = make_shared<constant_texture>(vec3(0.4, 0.4, 0.4));
	auto ground_texture = make_shared<checker_texture>(even_texture, odd_texture);

	auto ground = make_shared<lambertian>(ground_texture);
	objects.add(make_shared<xy_rect>(0, 500, 0, 500, -1, ground));

	// 墙面：使用不同颜色的Lambertian材质
	auto red_wall = make_shared<lambertian>(make_shared<constant_texture>(vec3(0.65, 0.05, 0.05)));
	auto green_wall = make_shared<lambertian>(make_shared<constant_texture>(vec3(0.12, 0.45, 0.15)));
	auto blue_wall = make_shared<lambertian>(make_shared<constant_texture>(vec3(0.05, 0.05, 0.65)));
	auto white = make_shared<lambertian>(make_shared<constant_texture>(vec3(0.73, 0.73, 0.73)));
	objects.add(make_shared<yz_rect>(0, 500, 0, 500, 500, green_wall));
	objects.add(make_shared<xz_rect>(0, 500, 0, 500, 500, red_wall));
	objects.add(make_shared<xz_rect>(0, 500, 0, 500, 0, blue_wall));
	objects.add(make_shared<yz_rect>(0, 500, 0, 500, 0, white));

	// 水晶吊灯：作为主光源，使用DiffuseLight材质
	auto chandelier_light = make_shared<diffuse_light>(make_shared<constant_texture>(vec3(7, 7, 7)));
	objects.add(make_shared<sphere>(vec3(250, 450, 250), 50, chandelier_light));

	// 镜面球体：展示光线追踪的反射效果
	auto mirror_sphere = make_shared<metal>(vec3(0.9, 0.9, 0.9), 0.0);
	objects.add(make_shared<sphere>(vec3(150, 150, 150), 50, mirror_sphere));
	objects.add(make_shared<sphere>(vec3(350, 150, 150), 50, mirror_sphere));

	// 水晶柱：使用透明材质展示折射效果
	auto glass_cylinder = make_shared<dielectric>(1.5);
	objects.add(make_shared<cylinder>(vec3(250, 100, 250), vec3(0, 1, 0), 50, 100, glass_cylinder));

	// 带有纹理的球体：展示纹理映射
	int nx, ny, nn;
	auto tex_data = stbi_load("earthmap.jpg", &nx, &ny, &nn, 0);
	auto textured_sphere_material = make_shared<lambertian>(make_shared<image_texture>(tex_data, nx, ny));
	objects.add(make_shared<sphere>(vec3(400, 200, 400), 100, textured_sphere_material));

	

	return objects;
}

hittable_list final_scene() {
	hittable_list boxes1;
	auto ground =
		make_shared<lambertian>(make_shared<constant_texture>(vec3(0.48, 0.83, 0.53)));

	const int boxes_per_side = 2;
	for (int i = 0; i < boxes_per_side; i++) {
		for (int j = 0; j < boxes_per_side; j++) {
			auto w = 100.0;
			auto x0 = -1000.0 + i * w;
			auto z0 = -1000.0 + j * w;
			auto y0 = 0.0;
			auto x1 = x0 + w;
			auto y1 = random_double(1, 101);
			auto z1 = z0 + w;

			boxes1.add(make_shared<box>(vec3(x0, y0, z0), vec3(x1, y1, z1), ground));
		}
	}

	hittable_list objects;

	objects.add(make_shared<bvh_node>(boxes1, 0, 1));

	auto light = make_shared<diffuse_light>(make_shared<constant_texture>(vec3(4, 4, 4)));
	objects.add(make_shared<xz_rect>(123, 423, 147, 412, 554, light));

	auto center1 = vec3(400, 400, 200);
	auto center2 = center1 + vec3(30, 0, 0);
	auto moving_sphere_material =
		make_shared<lambertian>(make_shared<constant_texture>(vec3(0.7, 0.3, 0.1)));
	objects.add(make_shared<moving_sphere>(center1, center2, 0, 1, 50, moving_sphere_material));

	objects.add(make_shared<sphere>(vec3(260, 150, 45), 50, make_shared<dielectric>(1.5)));
	objects.add(make_shared<sphere>(
		vec3(0, 150, 145), 50, make_shared<metal>(vec3(0.8, 0.8, 0.9), 10.0)
	));

	auto boundary = make_shared<sphere>(vec3(360, 150, 145), 70, make_shared<dielectric>(1.5));
	objects.add(boundary);
	objects.add(make_shared<constant_medium>(
		boundary, 0.2, make_shared<constant_texture>(vec3(0.2, 0.4, 0.9))
	));
	boundary = make_shared<sphere>(vec3(0, 0, 0), 5000, make_shared<dielectric>(1.5));
	objects.add(make_shared<constant_medium>(
		boundary, .0001, make_shared<constant_texture>(vec3(1, 1, 1))));

	int nx, ny, nn;
	auto tex_data = stbi_load("earthmap.jpg", &nx, &ny, &nn, 0);
	auto emat = make_shared<lambertian>(make_shared<image_texture>(tex_data, nx, ny));
	objects.add(make_shared<sphere>(vec3(400, 200, 400), 100, emat));
	auto pertext = make_shared<noise_texture>(0.1);
	objects.add(make_shared<sphere>(vec3(220, 280, 300), 80, make_shared<lambertian>(pertext)));

	hittable_list boxes2;
	auto white = make_shared<lambertian>(make_shared<constant_texture>(vec3(0.73, 0.73, 0.73)));
	int ns = 1000;
	for (int j = 0; j < ns; j++) {
		boxes2.add(make_shared<sphere>(random(0, 165), 10, white));
	}

	objects.add(make_shared<translate>(
		make_shared<rotate_y>(
			make_shared<bvh_node>(boxes2, 0.0, 1.0), 15),
		vec3(-100, 270, 395)
	)
	);

	return objects;
}

// 渲染函数，现在接受图像的一部分作为参数
void render_strip(int start_row, int end_row, const camera& cam, color background, const hittable& world, shared_ptr<hittable> lights) {
	for (int j = start_row; j < end_row; j++) {
		for (int i = 0; i < gWidth; i++) {
			color pixel_color(0, 0, 0);
			for (int s = 0; s < samples_per_pixel; ++s) {
				auto u = (i + random_double()) / (gWidth - 1);
				auto v = (j + random_double()) / (gHeight - 1);
				ray r = cam.get_ray(u, v);
				pixel_color += ray_color(r, background, world, lights, max_depth);
			}
			write_color(i, j, pixel_color);
		}
	}
}

void rendering()
{
	double startFrame = clock();

	printf("CGAssignment4 (built %s at %s) \n", __DATE__, __TIME__);
	std::cout << "Ray-tracing based rendering launched..." << std::endl;

	// Image
	
	const int image_width = gWidth;
	const int image_height = gHeight;
	const vec3 background(0, 0, 0);

	//hittable_list world = random_scene();
	//hittable_list world = two_spheres();
	//hittable_list world = two_perlin_spheres();
	//hittable_list world = earth();
	//hittable_list world = simple_light();
	


	auto lights = make_shared<hittable_list>();
	lights->add(make_shared<xz_rect>(213, 343, 227, 332, 554, shared_ptr<material>()));
	//lights->add(make_shared<sphere>(point3(190, 90, 190), 90, shared_ptr<material>()));
	lights->add(make_shared<sphere>(point3(100, 45, 100), 45, shared_ptr<material>()));
	lights->add(make_shared<sphere>(point3(500, 65, 300), 30, shared_ptr<material>()));

	hittable_list world = cornell_box();

	bvh_node tree(world, 0, 1);
	//hittable_list world = cornell_smoke();
	//hittable_list world = final_scene();

	// normal
	//vec3 lookfrom(13, 2, 3);
	//vec3 lookat(0, 0, 0);
	//vec3 vup(0, 1, 0);
	//auto dist_to_focus = 10.0;
	//auto aperture = 0.0;
	//auto vfov = 20;

	// light
	//vec3 lookfrom(26, 3, 12);
	//vec3 lookat(0, 2, 0);
	//vec3 vup(0, 1, 0);
	//auto dist_to_focus = 10.0;
	//auto aperture = 0.0;
	//auto vfov = 20.0;

	//vec3 lookfrom(30, 40, 100);
	//vec3 lookat(0, 22, 0);
	//vec3 vup(0, 1, 0);
	//auto dist_to_focus = 10.0;
	//auto aperture = 0.0;
	//auto vfov = 20.0;

	vec3 lookfrom(278, 278, -800);
	vec3 lookat(278, 278, 0);
	vec3 vup(0, 1, 0);
	auto dist_to_focus = 10.0;
	auto aperture = 0.0;
	auto vfov = 40.0;

	// 计算每个线程要渲染的行数
	int num_threads = std::thread::hardware_concurrency(); // 获取可用的CPU核心数
	int rows_per_thread = (gHeight + num_threads - 1) / num_threads;

	// 创建线程向量
	std::vector<std::thread> threads;

	camera cam(lookfrom, lookat, vup, vfov, aspect_ratio, aperture, dist_to_focus, 0.0, 1.0);

	// Render

	// 启动线程
	for (int t = 0; t < num_threads; ++t) {
		int start_row = t * rows_per_thread;
		int end_row = std::min(start_row + rows_per_thread, gHeight);
		threads.emplace_back(render_strip, start_row, end_row, cam, background, tree, lights);
	}

	// 等待所有线程完成
	for (auto& th : threads) {
		th.join();
	}


	double endFrame = clock();
	double timeConsuming = static_cast<double>(endFrame - startFrame) / CLOCKS_PER_SEC;
	std::cout << "Ray-tracing based rendering over..." << std::endl;
	std::cout << "The rendering task took " << timeConsuming << " seconds" << std::endl;
}
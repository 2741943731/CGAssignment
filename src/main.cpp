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
//#include "obj_mesh.h"

static std::vector<std::vector<color>> gCanvas;		//Canvas

// The width and height of the screen
const auto aspect_ratio = 8.0 / 8.0;
const int gWidth = 800;
const int gHeight = static_cast<int>(gWidth / aspect_ratio);

void rendering();

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

void write_color(int x, int y, color pixel_color, int samples_per_pixel)
{
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
	auto scale = 1.0 / samples_per_pixel;
	auto r = sqrt(scale * pixel_color.e[0]);
	auto g = sqrt(scale * pixel_color.e[1]);
	auto b = sqrt(scale * pixel_color.e[2]);
	color res(r, g, b);
	gCanvas[y][x] = res;

}

vec3 ray_color(const ray& r, const vec3& background, const hittable& world, int depth) {
	hit_record rec;

	// If we've exceeded the ray bounce limit, no more light is gathered.
	if (depth <= 0)
		return vec3(0, 0, 0);

	// If the ray hits nothing, return the background color.
	if (!world.hit(r, 0.001, infinity, rec))
		return background;

	ray scattered;
	vec3 attenuation;
	vec3 emitted = rec.mat_ptr->emitted(rec.u, rec.v, rec.p);
	if (!rec.mat_ptr->scatter(r, rec, attenuation, scattered))
		return emitted;

	return emitted + attenuation * ray_color(scattered, background, world, depth - 1);
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
	//std::vector<std::vector<double>> rotation = {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}};
	//vec3 move(0.0f, 20.0f, 0.0f);

	std::vector<std::vector<double>> rotation = {{5.0f, 0.0f, 0.0f}, {0.0f, 5.0f, 0.0f}, {0.0f, 0.0f, 5.0f}};
	vec3 move(400.0f, 550.0f, 80.0f);

	load_obj_model("../assets/SwordMinecraft.obj", make_shared<lambertian>(vec3(0.5, 0.7, 1.0)), objects, rotation, move);

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

	shared_ptr<hittable> box1 = make_shared<box>(vec3(0, 0, 0), vec3(165, 330, 165), white);
	box1 = make_shared<rotate_y>(box1, 15);
	box1 = make_shared<translate>(box1, vec3(265, 0, 295));
	objects.add(box1);

	shared_ptr<hittable> box2 = make_shared<box>(vec3(0, 0, 0), vec3(165, 165, 165), white);
	box2 = make_shared<rotate_y>(box2, -18);
	box2 = make_shared<translate>(box2, vec3(130, 0, 65));
	objects.add(box2);
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

	shared_ptr<hittable> box1 = make_shared<box>(vec3(0, 0, 0), vec3(165, 330, 165), white);
	box1 = make_shared<rotate_y>(box1, 15);
	box1 = make_shared<translate>(box1, vec3(265, 0, 295));

	shared_ptr<hittable> box2 = make_shared<box>(vec3(0, 0, 0), vec3(165, 165, 165), white);
	box2 = make_shared<rotate_y>(box2, -18);
	box2 = make_shared<translate>(box2, vec3(130, 0, 65));

	objects.add(
		make_shared<constant_medium>(box1, 0.01, make_shared<constant_texture>(vec3(0, 0, 0))));
	objects.add(
		make_shared<constant_medium>(box2, 0.01, make_shared<constant_texture>(vec3(1, 1, 1))));

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

void rendering()
{
	double startFrame = clock();

	printf("CGAssignment4 (built %s at %s) \n", __DATE__, __TIME__);
	std::cout << "Ray-tracing based rendering launched..." << std::endl;

	// Image
	const int samples_per_pixel = 10;
	const int image_width = gWidth;
	const int image_height = gHeight;
	const int max_depth = 20;
	auto R = cos(pi / 4);
	const auto aspect_ratio = double(image_width) / image_height;
	//const vec3 background(0.5, 0.7, 1.0);
	const vec3 background(0, 0, 0);

	//hittable_list world = random_scene();
	//hittable_list world = two_spheres();
	//hittable_list world = two_perlin_spheres();
	//hittable_list world = earth();
	//hittable_list world = simple_light();
	hittable_list world = cornell_box();
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

	camera cam(lookfrom, lookat, vup, vfov, aspect_ratio, aperture, dist_to_focus, 0.0, 1.0);

	// Render

	// The main ray-tracing based rendering loop
	// TODO: finish your own ray-tracing renderer according to the given tutorials
	for (int j = image_height - 1; j >= 0; j--)
	{
		for (int i = 0; i < image_width; i++)
		{
			color pixel_color(0, 0, 0);
			for (int s = 0; s < samples_per_pixel; ++s) {
				auto u = (i + random_double()) / image_width;
				auto v = (j + random_double()) / image_height;
				ray r = cam.get_ray(u, v);
				pixel_color += ray_color(r, background, world, max_depth);
			}
			write_color(i, j, pixel_color, samples_per_pixel);
		}
	}


	double endFrame = clock();
	double timeConsuming = static_cast<double>(endFrame - startFrame) / CLOCKS_PER_SEC;
	std::cout << "Ray-tracing based rendering over..." << std::endl;
	std::cout << "The rendering task took " << timeConsuming << " seconds" << std::endl;
}
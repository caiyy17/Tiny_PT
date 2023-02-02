#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{

    // Change the definition here to change resolution
    //Scene scene(784, 784);
    Scene scene(512, 512);

    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* mf1 = new Material(Microfacet, Vector3f(0.0f));
    mf1->Kd = Vector3f(0.7f);
    Material* mf2 = new Material(Microfacet, Vector3f(0.0f));
    mf2->Kd = Vector3f(0.7f);
    Material* mf3 = new Material(Microfacet, Vector3f(0.0f));
    mf3->Kd = Vector3f(0.7f);
    Material* mf4 = new Material(Microfacet, Vector3f(0.0f));
    mf4->Kd = Vector3f(0.7f);
    Material* light0 = new Material(DIFFUSE, Vector3f(100.0f));
    light0->Kd = Vector3f(0.65f);
    Material* light1 = new Material(DIFFUSE, Vector3f(729.0f));
    light1->Kd = Vector3f(0.65f);
    Material* light2 = new Material(DIFFUSE, Vector3f(81.0f));
    light2->Kd = Vector3f(0.65f);
    Material* light3 = new Material(DIFFUSE, Vector3f(9.0f));
    light3->Kd = Vector3f(0.65f);
    Material* light4 = new Material(DIFFUSE, Vector3f(1.0f));
    light4->Kd = Vector3f(0.65f);

    MeshTriangle floor("../models/veach_mi/floor.obj", red);
    MeshTriangle p1("../models/veach_mi/plate1.obj", mf1);
    p1.m->roughness = 0.001;
    MeshTriangle p2("../models/veach_mi/plate2.obj", mf2);
    p2.m->roughness = 0.005;
    MeshTriangle p3("../models/veach_mi/plate3.obj", mf3);
    p3.m->roughness = 0.02;
    MeshTriangle p4("../models/veach_mi/plate4.obj", mf4);
    p4.m->roughness = 0.1;
    Sphere s0(Vector3f(10,10,4), 2, light0);
    Sphere s1(Vector3f(-3.75,0,0), 0.033, light1);
    Sphere s2(Vector3f(-1.25,0,0), 0.1, light2);
    Sphere s3(Vector3f(1.25,0,0), 0.3, light3);
    Sphere s4(Vector3f(3.75,0,0), 0.9, light4);
    //MeshTriangle light_("../models/cornellbox/light.obj", light);

    //scene.Add(&floor);
    scene.Add(&p1);
    scene.Add(&p2);
    scene.Add(&p3);
    scene.Add(&p4);
    scene.Add(&s1);
    scene.Add(&s2);
    scene.Add(&s3);
    scene.Add(&s4);
    //scene.Add(&s0);

    //scene.Add(&light_);

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}
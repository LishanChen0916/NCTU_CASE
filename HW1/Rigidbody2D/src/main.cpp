/*

Computer animation assignment 1 : physical simulation

Please do not upload this homework to github.

Please fill your student ID : ""

First step : Look for TODO comments to find the methods that you need to implement.

*/

#include <iostream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <vector>

#include <GL/freeglut.h>

#include "linalg.h"

#include "clock.hpp"
#include "scene.hpp"
#include "circle.hpp"
#include "aabb.hpp"
#include "integrator.hpp"
#include "joint.hpp"

namespace
{
    const float deltaTime = 1.0f / 1000.0f;
    const uint32_t positional_correction_iterations = 10;
    const float accumulate_upper_bound = 
        std::max(deltaTime, 0.1f);

    auto integrator = std::make_shared<RungeKuttaFourthIntegrator>();
                      //std::make_shared<ExplicitEulerIntegrator>();

    auto scene = std::make_shared<Scene>(
        deltaTime, positional_correction_iterations, integrator);

    int screen_width = 600;
	int screen_height = 600;

    typedef linalg::aliases::float2 float2;
}

class GLUTCallback
{
private:
    static float accumulator;

    static void RenderScene()
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(
        0, 0, 0,
		0, 0, -1,
		0, 1, 0);

        scene->Render();

        glutSwapBuffers();
        glutPostRedisplay();
    }

public:
    static void MainLoop()
    {
        accumulator += (float)Clock::Elapsed();
        Clock::Reset();

        accumulator = std::clamp(accumulator, 0.0f, accumulate_upper_bound);
        while(accumulator >= deltaTime)
        {
            scene->Step();

            accumulator -= deltaTime;
        }

        RenderScene();
    }

    static void Keyboard(unsigned char key, int x, int y)
    {

    }

    static void Reshape(int width, int height)
    {
        screen_width = width;
        screen_height = height;

        glViewport( 0, 0, width, height );
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        float2 ortho_size((float)width / 20.0f, (float)height / 20.0f);
        
        gluOrtho2D(-ortho_size.x, ortho_size.x, -ortho_size.y, ortho_size.y);
    }

    static void Mouse(int button, int state, int x, int y)
    {
        if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
        {
            float2 ortho_size((float)screen_width / 20.0f, (float)screen_height / 20.0f);
            float2 position = float2( (float)x / 10.0f - ortho_size.x, (float)y / -10.0f + ortho_size.y );

            std::shared_ptr<Circle> shape = std::make_shared<Circle>(
                3.0f
            );
            auto body = scene->AddRigidBody(shape, position);
        }
        if(button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
        {
            float2 ortho_size((float)screen_width / 20.0f, (float)screen_height / 20.0f);
            float2 position = float2( (float)x / 10.0f - ortho_size.x, (float)y / -10.0f + ortho_size.y );

            std::shared_ptr<AABB> shape = std::make_shared<AABB>(
#ifdef _MSC_VER
				float2(((float)rand() / (RAND_MAX)) * 5 + 3, ((float)rand() / (RAND_MAX)) * 5 + 3)
#else
                float2( drand48() * 5 + 3, drand48() * 5 + 3 )
#endif
            );
            auto body = scene->AddRigidBody(shape, position);
        }
    }
};

float GLUTCallback::accumulator = 0.0f;

int main(int argc, char* argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(screen_width, screen_height);
    glutCreateWindow("PhyEngine");
    glutDisplayFunc(GLUTCallback::MainLoop);
    glutKeyboardFunc(GLUTCallback::Keyboard);
    glutMouseFunc(GLUTCallback::Mouse);
    glutReshapeFunc(GLUTCallback::Reshape);

	auto rk4 = std::dynamic_pointer_cast<RungeKuttaFourthIntegrator>(integrator);
	if (rk4)
	{
		rk4->scene = scene;
	}

    // fill in the scene
    // floor
    {
        std::shared_ptr<AABB> shape = std::make_shared<AABB>(
            float2 (35, 1)
        );

        auto body = scene->AddRigidBody(shape, float2(0, -10));
        // setting an infinite mass
        body->SetMass(0.0f);
    }
    {
        std::shared_ptr<Circle> shape = std::make_shared<Circle>(2.0f);
        auto body = scene->AddRigidBody(shape, float2(-12.5f, 5.0f));
        body->SetVelocity(float2(15, 0));
    }
    {
        std::shared_ptr<Circle> shape = std::make_shared<Circle>(2.0f);
        auto body = scene->AddRigidBody(shape, float2(-5, 16));
    }
    {
        std::shared_ptr<AABB> shape = std::make_shared<AABB>(
            float2 (5, 5)
        );
        auto body = scene->AddRigidBody(shape, float2(-5, 20));
        body->SetVelocity(float2(8, 5));
    }
    {
        const size_t box_size = 21;
		const float length = 25.0f;
		const float rest_length = (length / box_size);

        std::vector< std::shared_ptr<RigidBody2D> > boxes;
        boxes.reserve(box_size);

        float theta = 0.0f;
        float deltaTheta = (float) M_PI / (box_size - 1);

        for(size_t i = 0; i < box_size; i++)
        {
            auto shape = std::make_shared<AABB>(float2 (1, 1));
            boxes.push_back(scene->AddRigidBody(shape, 
                float2(length * std::cos(theta), -18.0f)
            ));
			boxes[i]->SetMass(1.0f);

            theta += deltaTheta;
        }

        boxes[0]->SetMass(0.0f);
        boxes[box_size - 1]->SetMass(0.0f);

        for(size_t i = 1; i < box_size; i++)
        {
            std::shared_ptr<SpringJoint> disJoint = 
                std::make_shared<SpringJoint>(boxes[i - 1], boxes[i], rest_length, 100.0f);
            scene->AddJoint(disJoint);
        }
    }
    {
        const size_t box_size = 7;
		const float length = 15.0f;
		const float rest_length = (length / box_size);

        std::vector< std::shared_ptr<RigidBody2D> > boxes;
        boxes.reserve(box_size);

        for(size_t i = 0; i < box_size; i++)
        {
            auto shape = std::make_shared<AABB>(float2 (1, 1));
            boxes.push_back(scene->AddRigidBody(shape, 
                float2( -20.0f + -3.0f * i, 30 - rest_length * i)
            ));
			boxes[i]->SetMass(1.0f);
            boxes[i]->SetVelocity(float2(1.0f, 0.0f));
        }

        boxes[0]->SetMass(0.0f);

        for(size_t i = 1; i < box_size; i++)
        {
            std::shared_ptr<DistanceJoint> disJoint = 
                std::make_shared<DistanceJoint>(boxes[i - 1], boxes[i], rest_length * 3.0f, deltaTime);
            scene->AddJoint(disJoint);
        }
    }
    
    glutMainLoop();

    return 0;
}
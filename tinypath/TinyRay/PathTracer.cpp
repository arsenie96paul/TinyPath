/*---------------------------------------------------------------------
*
* Copyright © 2015  Minsi Chen
* E-mail: m.chen@derby.ac.uk
*
* The source is written for the Graphics I and II modules. You are free
* to use and extend the functionality. The code provided here is functional
* however the author does not guarantee its performance.
---------------------------------------------------------------------*/
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#if defined(WIN32) || defined(_WINDOWS)
#include <Windows.h>
#include <gl/GL.h>
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#endif

#include "PathTracer.h"
#include "Ray.h"
#include "Scene.h"
#include "Camera.h"
#include "perlin.h"
#include "Primitive.h"

#define M_PI 3.14159265358979323846

PathTracer::PathTracer()
{
	m_buffHeight = m_buffWidth = 0.0;
	m_renderCount = 0;
	SetTraceLevel(5);
	m_traceflag = (TraceFlags)(TRACE_AMBIENT | TRACE_DIFFUSE_AND_SPEC |
		TRACE_SHADOW | TRACE_REFLECTION | TRACE_REFRACTION);
}

PathTracer::PathTracer(int Width, int Height)
{
	m_buffWidth = Width;
	m_buffHeight = Height;
	m_renderCount = 0;
	SetTraceLevel(5);

	m_framebuffer = new Framebuffer(Width, Height);

	//default set default trace flag, i.e. no lighting, non-recursive
	m_traceflag = (TraceFlags)(TRACE_AMBIENT);
}

PathTracer::~PathTracer()
{
	delete m_framebuffer;
}

inline double getUniformDouble()
{
	return (double)rand() / (double)RAND_MAX;
}

Colour PathTracer::TraceScene(Scene* pScene, Ray& ray, int tracelevel)
{
	RayHitResult result;
	result = pScene->IntersectByRay(ray);

	if (result.data) //the ray has hit something
	{

		// Colours diffuse and emissive
		Primitive* prim = (Primitive*)result.data;
		Material* mat = prim->GetMaterial();

		Colour emissive = mat->GetEmissiveColour(); // get the emissive colour
		Colour Diffuse = mat->GetDiffuseColour();  // get the diffuse colour

		double p = Diffuse[0] > Diffuse[1] && Diffuse[0] > Diffuse[2] ? Diffuse[0] : Diffuse[1] > Diffuse[2] ? Diffuse[1] : Diffuse[2]; // max reflectance 

		if (++tracelevel > 5)
		{
			if (getUniformDouble() < p) //throw a dice and decide if the trace should terminate
			{
				Diffuse = Diffuse*(1.0 / p);
			}
			else
			{
				return emissive; //R.R
			}
		}
		if (m_traceflag & TRACE_AMBIENT) 
		{
			Vector3 nl = result.normal;
			Vector3 n = nl.DotProduct(ray.GetRay()) < 0 ? nl : nl*-1;

			double r1 = 2 * M_PI*getUniformDouble(), r2 = getUniformDouble(), r2s = sqrt(r2);

			Vector3 u = (fabs(n[0]) > 0.1 ? Vector3(0, 1, 0) : Vector3(1, 0, 0)).CrossProduct(n).Normalise();
			Vector3 v = n.CrossProduct(u);
			Vector3 d = (u*cos(r1)*r2s + v *sin(r1)*r2s + n*sqrt(1 - r2)).Normalise();

			ray.SetRay(result.point + d *0.0001f, d);

			return emissive + Diffuse * this->TraceScene(pScene, ray, m_traceLevel);
		}
		if (m_traceflag & TRACE_REFLECTION)
		{
			Vector3 d = ray.GetRay().Reflect(result.normal).Normalise();
			ray.SetRay(result.point + d *0.0001f, d);
			return emissive + Diffuse * this->TraceScene(pScene, ray, --m_traceLevel);
		}
	}
	return pScene->GetBackgroundColour();
}

inline double clamp(double x)
{
	return x<0 ? 0 : x>1 ? 1 : x;
}

void PathTracer::DoPathTrace(Scene* pScene)
{
	Camera* cam = pScene->GetSceneCamera();

	Vector3 camRightVector = cam->GetRightVector();
	Vector3 camUpVector = cam->GetUpVector();
	Vector3 camViewVector = cam->GetViewVector();
	Vector3 centre = cam->GetViewCentre();
	Vector3 camPosition = cam->GetPosition();

	double sceneWidth = pScene->GetSceneWidth();
	double sceneHeight = pScene->GetSceneHeight();

	double pixelDX = sceneWidth / m_buffWidth;
	double pixelDY = sceneHeight / m_buffHeight;

	Vector3 start;

	start[0] = centre[0] - ((sceneWidth * camRightVector[0])
		+ (sceneHeight * camUpVector[0])) / 2.0;
	start[1] = centre[1] - ((sceneWidth * camRightVector[1])
		+ (sceneHeight * camUpVector[1])) / 2.0;
	start[2] = centre[2] - ((sceneWidth * camRightVector[2])
		+ (sceneHeight * camUpVector[2])) / 2.0;

	int samps = 25;             // number of samples

	if (m_renderCount == 0)
	{
		fprintf(stdout, "Trace start.\n");

		Colour colour;
		//TinyRay on multiprocessors using OpenMP!!!
#pragma omp parallel for schedule (dynamic, 1) private(colour)
		for (int y = 0; y < m_buffHeight; y++)
			for (int x = 0; x<m_buffWidth; x++)
				for (int sy = 0, i = (m_buffHeight - y - 1)*m_buffWidth + x; sy<2; sy++)     // 2x2 subpixel rows 
					for (int sx = 0; sx < 2; sx++, colour = Colour())						// 2x2 subpixel cols 
					{											
						for (int s = 0; s < samps; s++) // added loop for samples
						{
							Ray viewray;
							Vector3 pixel;

							pixel[0] = start[0] + (y + 0.5) * camUpVector[0] * pixelDY
								+ (x + 0.5) * camRightVector[0] * pixelDX;
							pixel[1] = start[1] + (y + 0.5) * camUpVector[1] * pixelDY
								+ (x + 0.5) * camRightVector[1] * pixelDX;
							pixel[2] = start[2] + (y + 0.5) * camUpVector[2] * pixelDY
								+ (x + 0.5) * camRightVector[2] * pixelDX;

							viewray.SetRay(camPosition, (pixel - camPosition).Normalise()); // set ray
							colour = colour + (this->TraceScene(pScene, viewray, 0))*(1. / samps); 
						}
						colour = Vector3(clamp(colour[0]), clamp(colour[1]), clamp(colour[2])) * 0.25; // clamping the colour
						m_framebuffer->WriteRGBToFramebuffer(colour, x, y);
					}
	}
	m_renderCount++;
}

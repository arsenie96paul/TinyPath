/*---------------------------------------------------------------------
*
* Copyright © 2016  Minsi Chen
* E-mail: m.chen@derby.ac.uk
*
* The source is written for the Graphics I and II modules. You are free
* to use and extend the functionality. The code provided here is functional
* however the author does not guarantee its performance.
---------------------------------------------------------------------*/
#include "Scene.h"
#include "Sphere.h"
#include "Plane.h"
#include "Box.h"

Scene::Scene()
{
	InitDefaultScene();
}


Scene::~Scene()
{
	CleanupScene();
}

/*---------------------------------------------------------------------
*
* Copyright © 2016  Minsi Chen
* E-mail: m.chen@derby.ac.uk
*
* The source is written for the Graphics I and II modules. You are free
* to use and extend the functionality. The code provided here is functional
* however the author does not guarantee its performance.
---------------------------------------------------------------------*/
void Scene::InitDefaultScene()
{
	//Create a box and its material
	Primitive* newobj = new Box(Vector3(-4.0, 4.0, -20.0), 10.0, 15.0, 4.0);
	Material* newmat = new Material();
	//mat for the box1
	newmat->SetAmbientColour(0.0, 0.0, 0.0);
	newmat->SetDiffuseColour(0.8, 0.0, 0.0);
	newmat->SetSpecularColour(1.0, 1.0, 1.0);
	newmat->SetSpecPower(20);
	newobj->SetMaterial(newmat);
	m_sceneObjects.push_back(newobj);
	m_objectMaterials.push_back(newmat);

	newobj = new Box(Vector3(4.0, 4.0, -15.0), 4.0, 20.0, 4.0);
	newmat = new Material();
	//mat for the box2
	newmat->SetAmbientColour(0.0, 0.0, 0.0);
	newmat->SetDiffuseColour(0.8, 0.8, 0.8);
	newmat->SetSpecularColour(1.0, 1.0, 1.0);
	newmat->SetSpecPower(20);
	newobj->SetMaterial(newmat);
	m_sceneObjects.push_back(newobj);
	m_objectMaterials.push_back(newmat);

	//light
	newobj = new Box(Vector3(0.0, 20.0, -10.0), 10.0, 4.0, 10.0);
	newmat = new Material();
	//mat for the box2
	newmat->SetAmbientColour(0.0, 0.0, 0.0);
	newmat->SetDiffuseColour(0.0, 0.0, 0.0);
	newmat->SetEmissiveColour(12, 12, 12); ///////////changes
	newmat->SetSpecularColour(1.0, 1.0, 1.0);
	newmat->SetSpecPower(20);
	newobj->SetMaterial(newmat);
	m_sceneObjects.push_back(newobj);
	m_objectMaterials.push_back(newmat);

	//Create sphere 1 and its material
	newobj = new Sphere(3.0, 2, -3.5, 2.0); //sphere 2
	newmat = new Material();
	newmat->SetAmbientColour(0.0, 0.0, 0.0);
	newmat->SetDiffuseColour(0.0, 0.8, 0.0);
	newmat->SetSpecularColour(1.0, 1.0, 1.0);
	newmat->SetSpecPower(2);
	newobj->SetMaterial(newmat);
	m_sceneObjects.push_back(newobj);
	m_objectMaterials.push_back(newmat);

	//Create sphere 2 and its material
	newobj = new Sphere(-2.0, 3.0, -5.0, 3.0); //sphere 3
	newmat = new Material();
	newmat->SetAmbientColour(0.0, 0.0, 0.0);
	newmat->SetDiffuseColour(0.0, 0.0, 0.8);
	newmat->SetSpecularColour(1.0, 1.0, 1.0);
	newmat->SetSpecPower(20);
	newobj->SetMaterial(newmat);
	m_sceneObjects.push_back(newobj);
	m_objectMaterials.push_back(newmat);


	newobj = new Plane(); //an xz plane at the origin, floor
	static_cast<Plane*>(newobj)->SetPlane(Vector3(0.0, 1.0, 0.0), 0.0);
	newmat = new Material();
	newmat->SetAmbientColour(0.0, 0.0, 0.0);
	newmat->SetDiffuseColour(0.7, 0.7, 0.7);
	newmat->SetSpecularColour(0.0, 0.0, 0.0);
	newmat->SetSpecPower(10);
	newmat->SetCastShadow(false);
	newobj->SetMaterial(newmat);
	m_sceneObjects.push_back(newobj);
	m_objectMaterials.push_back(newmat);

	newobj = new Plane(); //an xz plane 40 units above, ceiling
	static_cast<Plane*>(newobj)->SetPlane(Vector3(0.0, -1.0, 0.0), -20.0);
	newobj->SetMaterial(newmat);
	m_sceneObjects.push_back(newobj);

	//material for front and back walls
	newmat = new Material();
	newmat->SetAmbientColour(0.0, 0.0, 0.0);
	newmat->SetDiffuseColour(0.0, 0.7, 0.0);
	newmat->SetSpecularColour(0.0, 0.0, 0.0);
	newmat->SetSpecPower(10);
	newmat->SetCastShadow(false);

	newobj = new Plane(); //an xy plane 40 units along -z axis, 
	static_cast<Plane*>(newobj)->SetPlane(Vector3(0.0, 0.0, 1.0), -40.0);
	m_sceneObjects.push_back(newobj);
	newobj->SetMaterial(newmat);

	newobj = new Plane(); //an xy plane 40 units along the z axis
	static_cast<Plane*>(newobj)->SetPlane(Vector3(0.0, 0.0, -1.0), -40.0);
	m_sceneObjects.push_back(newobj);
	newobj->SetMaterial(newmat);

	newobj = new Plane(); //an yz plane 20 units along -x axis
	static_cast<Plane*>(newobj)->SetPlane(Vector3(1.0, 0.0, 0.0), -20.0);
	newmat = new Material();
	newmat->SetAmbientColour(0.0, 0.0, 0.0);
	newmat->SetDiffuseColour(0.0, 0.0, 0.7);
	newmat->SetSpecularColour(0.0, 0.0, 0.0);
	newmat->SetSpecPower(10);
	newmat->SetCastShadow(false);
	newobj->SetMaterial(newmat);
	m_sceneObjects.push_back(newobj);
	m_objectMaterials.push_back(newmat);

	newobj = new Plane(); //an yz plane 20 units along +x axis
	static_cast<Plane*>(newobj)->SetPlane(Vector3(-1.0, 0.0, 0.0), -20.0);
	newobj->SetMaterial(newmat);
	m_sceneObjects.push_back(newobj);

	//Create one light source for the scene
	Light *newlight = new Light();
	newlight->SetLightPosition(0.0, 15.0, 10.0);
	m_lights.push_back(newlight);

	//A blue background
	m_background.SetVector(0.25, 0.6, 1.0);

	//default scene width and height;
	m_sceneWidth = 1.33333333;
	m_sceneHeight = 1.0;

	//default camera position and look at
	m_activeCamera.SetPositionAndLookAt(Vector3(0.0, 7.0, 13.0), Vector3(0.0, 7.0, 0.0));
}

void Scene::CleanupScene()
{
	//Cleanup object list
	std::vector<Primitive*>::iterator prim_iter = m_sceneObjects.begin();

	while(prim_iter != m_sceneObjects.end())
	{
		delete *prim_iter;
		prim_iter++;
	}

	m_sceneObjects.clear();

	//Cleanup material list
	std::vector<Material*>::iterator mat_iter = m_objectMaterials.begin();

	while (mat_iter != m_objectMaterials.end())
	{
		delete *mat_iter;
		mat_iter++;
	}
	m_objectMaterials.clear();

	//cleanup light list
	std::vector<Light*>::iterator lit_iter = m_lights.begin();

	while (lit_iter != m_lights.end())
	{
		delete *lit_iter;
		lit_iter++;
	}

	m_lights.clear();
}

RayHitResult Scene::IntersectByRay(Ray& ray)
{
	RayHitResult result = Ray::s_defaultHitResult;
	
	//TODO: Implement ray to scene intersection
	//All geometric objects are stored in m_sceneObjects.
	//To find out which object intersects with the ray
	//1. Enumerate through each primitive in m_sceneObjects
	//2. For each primitive, there is an IntersectByRay method which can be called to test for intersection
	//
	//Remember: a ray can potentially intersect with more than one object
	//We are only interested in the closest one, i.e. one with the smallest positive t
	//Check result.t for the parametric result of the interesection

	RayHitResult CurrentResult;

	for each (Primitive* prim in m_sceneObjects)
	{
		CurrentResult =prim->IntersectByRay(ray);

		if (CurrentResult.t < 0.0)
		{
			continue;
		}
		if (CurrentResult.t < result.t)
		{
			result = CurrentResult;
		}
	}
	return result;
}

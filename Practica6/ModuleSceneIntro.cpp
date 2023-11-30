#include "Globals.h"
#include "Application.h"
#include "ModuleRender.h"
#include "ModuleSceneIntro.h"
#include "ModuleInput.h"
#include "ModuleTextures.h"
#include "ModuleAudio.h"
#include "ModulePhysics.h"

ModuleSceneIntro::ModuleSceneIntro(Application* app, bool start_enabled) : Module(app, start_enabled)
{
	circle = box = rick = NULL;
	ray_on = false;
	sensed = false;
}

ModuleSceneIntro::~ModuleSceneIntro()
{}

// Load assets
bool ModuleSceneIntro::Start()
{
	LOG("Loading Intro assets");
	bool ret = true;

	App->renderer->camera.x = App->renderer->camera.y = 0;

	circle = App->textures->Load("pinball/wheel.png"); 
	box = App->textures->Load("pinball/crate.png");
	rick = App->textures->Load("pinball/rick_head.png");
	bonus_fx = App->audio->LoadFx("pinball/bonus.wav");

	circles.add(App->physics->CreateCircle(SCREEN_WIDTH / 2, SCREEN_HEIGHT/2, 200));
	circles.getLast()->data->listener = this;
	circles.getLast()->data->body->SetGravityScale(0);
	//circles.getLast()->data->body->SetType(b2_kinematicBody);
	circles.getLast()->data->body->GetFixtureList()->SetDensity(10000000000);
	

	

	/**/

	/*mainPlanet = App->physics->CreateCircle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, 200);
	mainPlanet->listener = this;
	mainPlanet->body->SetGravityScale(0);
	mainPlanet->body->SetType(b2_staticBody);*/

	mainPlanet_transform = iPoint(circles.getLast()->data->body->GetTransform().p.x, circles.getLast()->data->body->GetTransform().p.y);


	return ret;
}

// Load assets
bool ModuleSceneIntro::CleanUp()
{
	LOG("Unloading Intro scene");

	return true;
}

// Update: draw background
update_status ModuleSceneIntro::Update()
{
	
	if (test) {
		circles.add(App->physics->CreateCircle(SCREEN_WIDTH / 1.2f, SCREEN_HEIGHT / 2, 25));
		circles.getLast()->data->listener = this;
		circles.getLast()->data->body->SetGravityScale(0);
		circles.getLast()->data->body->GetFixtureList()->SetDensity(1);
		circles.getLast()->data->body->ApplyForce(b2Vec2(0, 300), circles.getLast()->data->body->GetWorldCenter(), true);



		circles.add(App->physics->CreateCircle(SCREEN_WIDTH / 1.2f + 35, SCREEN_HEIGHT / 2, 3));
		circles.getLast()->data->listener = this;
		circles.getLast()->data->body->SetGravityScale(0);
		circles.getLast()->data->body->GetFixtureList()->SetDensity(1);
		circles.getLast()->data->body->ApplyForce(b2Vec2(0.2f, 2.6f), circles.getLast()->data->body->GetWorldCenter(), true);
		


		test = false;
	}



	if(App->input->GetKey(SDL_SCANCODE_1) == KEY_DOWN)
	{
		circles.add(App->physics->CreateCircle(App->input->GetMouseX(), App->input->GetMouseY(), 25));
		circles.getLast()->data->listener = this;
		circles.getLast()->data->body->SetGravityScale(0);
		circles.getLast()->data->body->GetFixtureList()->SetDensity(1);
	}

	


	for (int i = 0; i < circles.count(); i++) {

		PhysBody* circleA;
		circles.at(i, circleA);

		b2Body* bodyA = circleA->body;
		b2Vec2 pivoteA = bodyA->GetWorldCenter();
		float masaA = bodyA->GetMass();

		if (circles.count() > 1) {

			for (int j = i+1; j < circles.count(); j++) {

				PhysBody* circleB;
				circles.at(j, circleB);

				b2Body* bodyB = circleB->body;
				b2Vec2 pivoteB = bodyB->GetWorldCenter();
				float masaB = bodyB->GetMass();
				b2Vec2 delta = pivoteB - pivoteA;
				float r = delta.Length();
				float force = App->physics->G * masaA * masaB / (r * r);
				
				delta.Normalize();
				//force = 10;
				bodyA->ApplyForce(force * delta, pivoteA, true);
				bodyB->ApplyForce(-force * delta, pivoteB, true);
				

			}
		}
	}


	PhysBody* circlePlaneta;
	circles.at(0, circlePlaneta);
	circlePlaneta->body->SetTransform(b2Vec2(mainPlanet_transform.x, mainPlanet_transform.y), 0);

	/*for (int i = 0; i < circles.count(); i++) {
		PhysBody* circle;
		circles.at(i, circle);

		b2Body* planeta = mainPlanet->body;
		b2Vec2 pivotPlaneta = planeta->GetWorldCenter();
		float masaPlaneta = planeta->GetMass();

		b2Body* luna = circle->body;
		b2Vec2 pivotLuna = luna->GetWorldCenter();
		float masaLuna = luna->GetMass();

		b2Vec2 dist = pivotLuna - pivotPlaneta;
		float r = dist.Length();
		float force = App->physics->G * masaPlaneta * masaLuna / (r * r);
		force = 10;
		dist.Normalize();
		planeta->ApplyForce(force * dist, pivotPlaneta, true);
		luna->ApplyForce(-force * dist, pivotLuna, true);
	}*/







	

	// All draw functions ------------------------------------------------------
	p2List_item<PhysBody*>* c = circles.getFirst();

	while(c != NULL)
	{
		int x, y;
		c->data->GetPosition(x, y);
		if(c->data->Contains(App->input->GetMouseX(), App->input->GetMouseY()))
			App->renderer->Blit(circle, x, y, NULL, 1.0f, c->data->GetRotation());
		c = c->next;
	}

	c = boxes.getFirst();

	

	

	return UPDATE_CONTINUE;
}

void ModuleSceneIntro::OnCollision(PhysBody* bodyA, PhysBody* bodyB)
{
	int x, y;

	App->audio->PlayFx(bonus_fx);

	/*
	if(bodyA)
	{
		bodyA->GetPosition(x, y);
		App->renderer->DrawCircle(x, y, 50, 100, 100, 100);
	}

	if(bodyB)
	{
		bodyB->GetPosition(x, y);
		App->renderer->DrawCircle(x, y, 50, 100, 100, 100);
	}*/
}

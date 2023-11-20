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
	circles.getLast()->data->body->GetFixtureList()->SetDensity(10000000000000);
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
	

	if(App->input->GetKey(SDL_SCANCODE_1) == KEY_DOWN)
	{
		circles.add(App->physics->CreateCircle(App->input->GetMouseX(), App->input->GetMouseY(), 25));
		circles.getLast()->data->listener = this;
		circles.getLast()->data->body->SetGravityScale(0);
		circles.getLast()->data->body->GetFixtureList()->SetDensity(10);
	}

	


	for (int i = 0; i < circles.count(); i++) {

		PhysBody* circle;
		circles.at(i, circle);

		b2Body* bi = circle->body;
		b2Vec2 pi = bi->GetWorldCenter();
		float mi = bi->GetMass();
		for (int k = i+1; k < circles.count(); k++) {

			PhysBody* circleK;
			circles.at(k, circleK);

			b2Body* bk = circleK->body;
			b2Vec2 pk = bk->GetWorldCenter();
			float mk = bk->GetMass();
			b2Vec2 delta = pk - pi;
			float r = delta.Length();
			float force = App->physics->G * mi * mk / (r * r);
			delta.Normalize();
			force = 10;
			bi->ApplyForce(force * delta, pi, true);
			bk->ApplyForce(-force * delta, pk, true);
		}
	}






	

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

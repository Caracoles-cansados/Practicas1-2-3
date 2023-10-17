#include "Globals.h"
#include "Application.h"
#include "ModulePhysics.h"
#include "math.h"
#include <stdio.h>
#include <string>
#include <string.h>
#include <iostream>
#include <cmath>
#include <stack>

// TODO 1: Include Box 2 header and library

ModulePhysics::ModulePhysics(Application* app, bool start_enabled) : Module(app, start_enabled)
{
	debug = true;
}

// Destructor
ModulePhysics::~ModulePhysics()
{
}

bool ModulePhysics::Start()
{
	LOG("Creating Physics 2D environment");

	// Create ground
	ground = Ground();
	ground.x = 0.0f; // [m]
	ground.y = 0.0f; // [m]
	ground.w = 30.0f; // [m]
	ground.h = 15.0f; // [m]

	box = Ground();
	box.x = 43.0f; // [m]
	box.y = 3.0f; // [m]
	box.w = 4.0f; // [m]
	box.h = 400.0f; // [m]

	

	// Create Water
	water = Water();
	water.x = ground.x + ground.w; // Start where ground ends [m]
	water.y = 0.0f; // [m]
	water.w = 30.0f; // [m]
	water.h = 5.0f; // [m]
	water.density = 50.0f; // [kg/m^3]
	water.vx = -1.0f; // [m/s]
	water.vy = 0.0f; // [m/s]

	// Create atmosphere
	atmosphere = Atmosphere();
	atmosphere.windx = -5.0f; // [m/s]
	atmosphere.windy = 5.0f; // [m/s]
	atmosphere.density = 1.0f; // [kg/m^3]

	// Create a ball
	PhysBall ball = PhysBall();

	// Set static properties of the ball
	ball.mass = 10.0f; // [kg]
	ball.surface = 1.0f; // [m^2]
	ball.radius = 0.5f; // [m]
	ball.cd = 0.4f; // [-]
	ball.cl = 1.2f; // [-]
	ball.b = 10.0f; // [...]
	ball.coef_friction = 0.9f; // [-]
	ball.coef_restitution = 0.8f; // [-]

	// Set initial position and velocity of the ball
	ball.x = 2.0f;
	ball.y = (ground.y + ground.h) + 2.0f;
	ball.vx = 0.0f;
	ball.vy = 0.0f;

	// Add ball to the collection
	//balls.emplace_back(ball);

	pinballBox = PinballBox();
	pinballBox.x = 20;
	pinballBox.y = 30;
	pinballBox.w = 5;
	pinballBox.h = 5;
	
	pinballBox.UpdateCollisions();


	pinballBox2 = PinballBox();
	pinballBox2.x = 30;
	pinballBox2.y = 0;
	pinballBox2.w = 5;
	pinballBox2.h = 500;

	pinballBox2.UpdateCollisions();



	App->font->AddFonts();

	return true;
}

update_status ModulePhysics::PreUpdate()
{
	float deltapain = 0.8f;

	if (App->input->GetKey(SDL_SCANCODE_A) == KEY_REPEAT) {
		angulo -= dt * deltapain;
	}
	if (App->input->GetKey(SDL_SCANCODE_D) == KEY_REPEAT) {
		angulo += dt * deltapain;
	}
	if (App->input->GetKey(SDL_SCANCODE_W) == KEY_REPEAT) {
		potencia += dt * deltapain * 50;
	}
	if (App->input->GetKey(SDL_SCANCODE_S) == KEY_REPEAT) {
		potencia -= dt * deltapain * 50;
	}

	


	if (App->input->GetKey(SDL_SCANCODE_SPACE) == KEY_DOWN) {
		PhysBall ball = PhysBall();
		atocao = false;
		// Set static properties of the ball
		ball.mass = 10.0f; // [kg]
		ball.surface = 1.0f; // [m^2]
		ball.radius = 0.5f; // [m]
		ball.cd = 0.4f; // [-]
		ball.cl = 1.2f; // [-]
		ball.b = 10.0f; // [...]
		ball.coef_friction = 0.9f; // [-]
		ball.coef_restitution = 0.8f; // [-]

		// Set initial position and velocity of the ball
		ball.x = 2.0f;
		ball.y = (ground.y + ground.h) + 2.0f;
		ball.vx = potencia * cos(-angulo);
		ball.vy = potencia * sin(-angulo);

		atmosphere.windx = 0.0f; // [m/s]
		atmosphere.windy = 0.0f; // [m/s]

		balls.emplace_back(ball);
		if (balls.size() > 1) {
			//balls.pop_front();
		}

	}


	// Process all balls in the scenario
	for (auto& ball : balls)
	{
		// Skip ball if physics not enabled
		if (!ball.physics_enabled)
		{
			continue;
		}

		


		
		// Step #0: Clear old values
		// ----------------------------------------------------------------------------------------
		
		// Reset total acceleration and total accumulated force of the ball
		ball.fx = ball.fy = 0.0f;
		ball.ax = ball.ay = 0.0f;

		// Step #1: Compute forces
		// ----------------------------------------------------------------------------------------

		// Gravity force
		float fgx = ball.mass * 0.0f;
		float fgy = ball.mass * -10.0f; // Let's assume gravity is constant and downwards
		ball.fx += fgx; ball.fy += fgy; // Add this force to ball's total force

		// Aerodynamic Drag force (only when not in water)
		if (!is_colliding_with_water(ball, water))
		{
			float fdx = 0.0f; float fdy = 0.0f;
			compute_aerodynamic_drag(fdx, fdy, ball, atmosphere);
			ball.fx += fdx; ball.fy += fdy; // Add this force to ball's total force
		}

		// Hydrodynamic forces (only when in water)
		if (is_colliding_with_water(ball, water))
		{
			// Hydrodynamic Drag force
			float fhdx = 0.0f; float fhdy = 0.0f;
			compute_hydrodynamic_drag(fhdx, fhdy, ball, water);
			ball.fx += fhdx; ball.fy += fhdy; // Add this force to ball's total force

			// Hydrodynamic Buoyancy force
			float fhbx = 0.0f; float fhby = 0.0f;
			compute_hydrodynamic_buoyancy(fhbx, fhby, ball, water);
			ball.fx += fhbx; ball.fy += fhby; // Add this force to ball's total force
		}

		// Other forces
		// ...

		// Step #2: 2nd Newton's Law
		// ----------------------------------------------------------------------------------------
		
		// SUM_Forces = mass * accel --> accel = SUM_Forces / mass
		ball.ax = ball.fx / ball.mass;
		ball.ay = ball.fy / ball.mass;

		// Step #3: Integrate --> from accel to new velocity & new position
		// ----------------------------------------------------------------------------------------

		// We will use the 2nd order "Velocity Verlet" method for integration.
		integrator_velocity_verlet(ball, dt);

		// Step #4: solve collisions
		// ----------------------------------------------------------------------------------------

		// Solve collision between ball and ground
		if (is_colliding_with_ground(ball, ground))
		{
			

			// TP ball to ground surface
			ball.y = ground.y + ground.h + ball.radius;

			// Elastic bounce with ground
			ball.vy = - ball.vy;

			// FUYM non-elasticity
			ball.vx *= ball.coef_friction;
			ball.vy *= ball.coef_restitution;
		} 

		if (is_colliding_with_ground(ball, box)) {
			if (is_colliding_with_ground(ball, box)) {
				atocao = true;
			}
			// TP ball to ground surface
			ball.y = box.y + box.h + ball.radius;

			// Elastic bounce with ground
			ball.vy = -ball.vy;

			// FUYM non-elasticity
			ball.vx *= ball.coef_friction;
			ball.vy *= ball.coef_restitution;
		}

		if (int orientacion = is_colliding_with_pinballBox(ball, pinballBox)) {

			switch (orientacion)
			{
			case 1:
				ball.y = pinballBox.top.y + pinballBox.top.h + ball.radius;
				ball.vy = -ball.vy;

				break;
			case 2:
				ball.x = pinballBox.rigth.x + pinballBox.rigth.w + ball.radius;
				ball.vx = -ball.vx;

				break;
			case 3:
				ball.y = pinballBox.down.y - pinballBox.down.h - ball.radius;
				ball.vy = -ball.vy;
				break;
			case 4:
				ball.x = pinballBox.left.x - ball.radius;
				ball.vx = -ball.vx;
				break;

			default:
				break;
			}

			ball.vx *= ball.coef_friction;
			ball.vy *= ball.coef_restitution;


		}

		if (int orientacion = is_colliding_with_pinballBox(ball, pinballBox2)) {

			switch (orientacion)
			{
			case 1:
				ball.y = pinballBox2.top.y + pinballBox2.top.h + ball.radius;
				ball.vy = -ball.vy;

				break;
			case 2:
				ball.x = pinballBox2.rigth.x + pinballBox2.rigth.w + ball.radius;
				ball.vx = -ball.vx;

				break;
			case 3:
				ball.y = pinballBox2.down.y - pinballBox2.down.h - ball.radius;
				ball.vy = -ball.vy;
				break;
			case 4:
				//ball.x = pinballBox.left.x - pinballBox.left.w - ball.radius;
				ball.x = pinballBox2.left.x - pinballBox2.left.w - ball.radius;
				ball.vx = -ball.vx;
				break;

			default:
				break;
			}

			ball.vx *= ball.coef_friction;
			ball.vy *= ball.coef_restitution;


		}


	}

	// Continue game
	return UPDATE_CONTINUE;
}

update_status ModulePhysics::PostUpdate()
{
	std::string mensajito = "potencia " + std::string(std::to_string(potencia));
	std::string mensajito2 = "angulo " + std::string(std::to_string(angulo));
	std::string mensajito3 = "viento x " + std::string(std::to_string(atmosphere.windx));
	std::string mensajito4 = "viento y " + std::string(std::to_string(atmosphere.windy));

	App->font->BlitText(100, 100, App->scoreFont, mensajito.c_str());
	App->font->BlitText(100, 120, App->scoreFont, mensajito2.c_str());
	App->font->BlitText(100, 160, App->scoreFont, mensajito3.c_str());
	App->font->BlitText(100, 180, App->scoreFont, mensajito4.c_str());



	// Colors
	int color_r, color_g, color_b;

	// Draw ground
	color_r = 0; color_g = 255; color_b = 0;
	App->renderer->DrawQuad(ground.pixels(), color_r, color_g, color_b);
	
	color_r = 255; color_g = 120; color_b = 120;
	SDL_SetRenderDrawColor(App->renderer->renderer,color_r, color_g, color_b, 255);
	SDL_RenderDrawLine(App->renderer->renderer, 42.0f,420, (potencia*5 * cos(angulo)) + 42, (potencia*5 * sin(angulo)) + 420);



	


	// Draw water
	color_r = 0; color_g = 0; color_b = 255;
	App->renderer->DrawQuad(water.pixels(), color_r, color_g, color_b, 200);

	if (!atocao) {
		color_r = 255; color_g = 0; color_b = 0;
	}
	else {
		color_r = 0; color_g = 255; color_b = 0;
	}
	//App->renderer->DrawQuad({ 850, 628, 40, 40 }, color_r, color_g, color_b);
	App->renderer->DrawQuad(box.pixels(), color_r, color_g, color_b);



	//DRAW PINBALL BOX
	pinballBox.UpdateCollisions();

	App->renderer->DrawQuad(pinballBox.top.pixels(), 136, 208, 37);
	App->renderer->DrawQuad(pinballBox.left.pixels(), 255, 255, 0);
	App->renderer->DrawQuad(pinballBox.down.pixels(), 255, 255, 255);
	App->renderer->DrawQuad(pinballBox.rigth.pixels(), 8, 222, 166);


	pinballBox2.UpdateCollisions();

	App->renderer->DrawQuad(pinballBox2.top.pixels(), 136, 208, 37);
	App->renderer->DrawQuad(pinballBox2.left.pixels(), 255, 255, 0);
	App->renderer->DrawQuad(pinballBox2.down.pixels(), 255, 255, 255);
	App->renderer->DrawQuad(pinballBox2.rigth.pixels(), 8, 222, 166);
	
	

	// Draw all balls in the scenario
	for (auto& ball : balls)
	{
		// Convert from physical magnitudes to geometrical pixels
		int pos_x = METERS_TO_PIXELS(ball.x);
		int pos_y = SCREEN_HEIGHT - METERS_TO_PIXELS(ball.y);
		int size_r = METERS_TO_PIXELS(ball.radius);

		// Select color
		if (ball.physics_enabled)
		{
			color_r = 255; color_g = 255; color_b = 255;
		}
		else
		{
			color_r = 255; color_g = 0; color_b = 0;
		}

		// Draw ball
		App->renderer->DrawCircle(pos_x, pos_y, size_r, color_r, color_g, color_b);
	}

	return UPDATE_CONTINUE;
}

// Called before quitting
bool ModulePhysics::CleanUp()
{
	return true;
}

// Compute modulus of a vector
float modulus(float vx, float vy)
{
	return std::sqrt(vx * vx + vy * vy);
}

// Compute Aerodynamic Drag force
void compute_aerodynamic_drag(float &fx, float& fy, const PhysBall &ball, const Atmosphere &atmosphere)
{
	float rel_vel[2] = { ball.vx - atmosphere.windx, ball.vy - atmosphere.windy }; // Relative velocity
	float speed = modulus(rel_vel[0], rel_vel[1]); // Modulus of the relative velocity
	float rel_vel_unitary[2] = { rel_vel[0] / speed, rel_vel[1] / speed }; // Unitary vector of relative velocity
	float fdrag_modulus = 0.5f * atmosphere.density * speed * speed * ball.surface * ball.cd; // Drag force (modulus)
	fx = -rel_vel_unitary[0] * fdrag_modulus; // Drag is antiparallel to relative velocity
	fy = -rel_vel_unitary[1] * fdrag_modulus; // Drag is antiparallel to relative velocity
}

// Compute Hydrodynamic Drag force
void compute_hydrodynamic_drag(float& fx, float& fy, const PhysBall& ball, const Water& water)
{
	float rel_vel[2] = { ball.vx - water.vx, ball.vy - water.vy }; // Relative velocity
	float speed = modulus(rel_vel[0], rel_vel[1]); // Modulus of the relative velocity
	float rel_vel_unitary[2] = { rel_vel[0] / speed, rel_vel[1] / speed }; // Unitary vector of relative velocity
	float fdrag_modulus = ball.b * speed; // Drag force (modulus)
	fx = -rel_vel_unitary[0] * fdrag_modulus; // Drag is antiparallel to relative velocity
	fy = -rel_vel_unitary[1] * fdrag_modulus; // Drag is antiparallel to relative velocity
}

// Compute Hydrodynamic Buoyancy force
void compute_hydrodynamic_buoyancy(float& fx, float& fy, const PhysBall& ball, const Water& water)
{
	// Compute submerged area (assume ball is a rectangle, for simplicity)
	float water_top_level = water.y + water.h; // Water top level y
	float h = 2.0f * ball.radius; // Ball "hitbox" height
	float surf = h * (water_top_level - ball.y); // Submerged surface
	if ((ball.y + ball.radius) < water_top_level) surf = h * h; // If ball completely submerged, use just all ball area
	surf *= 0.4; // FUYM to adjust values (should compute the area of circle segment correctly instead; I'm too lazy for that)

	// Compute Buoyancy force
	double fbuoyancy_modulus = water.density * 10.0 * surf; // Buoyancy force (modulus)
	fx = 0.0; // Buoyancy is parallel to pressure gradient
	fy = fbuoyancy_modulus; // Buoyancy is parallel to pressure gradient
}

// Integration scheme: Velocity Verlet
void integrator_velocity_verlet(PhysBall& ball, float dt)
{
	ball.x += ball.vx * dt + 0.5f * ball.ax * dt * dt;
	ball.y += ball.vy * dt + 0.5f * ball.ay * dt * dt;
	ball.vx += ball.ax * dt;
	ball.vy += ball.ay * dt;
}

// Detect collision with ground
bool is_colliding_with_ground(const PhysBall& ball, const Ground& ground)
{
	float rect_x = (ground.x + ground.w / 2.0f); // Center of rectangle
	float rect_y = (ground.y + ground.h / 2.0f); // Center of rectangle
	return check_collision_circle_rectangle(ball.x, ball.y, ball.radius, rect_x, rect_y, ground.w, ground.h);
}

// Detect collision with water
bool is_colliding_with_water(const PhysBall& ball, const Water& water)
{
	float rect_x = (water.x + water.w / 2.0f); // Center of rectangle
	float rect_y = (water.y + water.h / 2.0f); // Center of rectangle
	return check_collision_circle_rectangle(ball.x, ball.y, ball.radius, rect_x, rect_y, water.w, water.h);
}

// Detect collision between circle and rectange
bool check_collision_circle_rectangle(float cx, float cy, float cr, float rx, float ry, float rw, float rh)
{
	// Algorithm taken from https://stackoverflow.com/a/402010

	// Distance from center of circle to center of rectangle
	float dist_x = std::abs(cx - rx);
	float dist_y = std::abs(cy - ry);

	// If circle is further than half-rectangle, not intersecting
	if (dist_x > (rw / 2.0f + cr)) { return false; }
	if (dist_y > (rh / 2.0f + cr)) { return false; }

	// If circle is closer than half-rectangle, is intersecting
	if (dist_x <= (rw / 2.0f)) { return true; }
	if (dist_y <= (rh / 2.0f)) { return true; }

	// If all of above fails, check corners
	float a = dist_x - rw / 2.0f;
	float b = dist_y - rh / 2.0f;
	float cornerDistance_sq = a * a + b * b;
	return (cornerDistance_sq <= (cr * cr));
}

// Convert from meters to pixels (for SDL drawing)
SDL_Rect Ground::pixels()
{
	SDL_Rect pos_px{};
	pos_px.x = METERS_TO_PIXELS(x);
	pos_px.y = SCREEN_HEIGHT - METERS_TO_PIXELS(y);
	pos_px.w = METERS_TO_PIXELS(w);
	pos_px.h = METERS_TO_PIXELS(-h); // Can I do this? LOL
	return pos_px;
}


// Detect collision with ground
int is_colliding_with_pinballBox(const PhysBall& ball, const PinballBox& pinballbox)
{
	int ret = 0;
	// 1 -> arriba,  2 -> derecha,  3 -> abajo,  4 -> izquierda 


	
	if (is_colliding_with_ground(ball, pinballbox.top)) {
		ret = 1;
	}

	if (is_colliding_with_ground(ball, pinballbox.rigth)) {
		ret = 2;
	}

	if (is_colliding_with_ground(ball, pinballbox.down)) {
		ret = 3;
	}

	if (is_colliding_with_ground(ball, pinballbox.left)) {
		ret = 4;
	}
	



	return ret;
}


void mostrarTextos() {
	
}

/*class PinballBox : public Ground {
	
public:
	Ground top;
	Ground down;
	Ground left;
	Ground rigth;

	float x, y, w, h;

	PinballBox(float x, float y, float w, float h);

	bool UpdateCollisions();
	bool DrawColor();

};*/

PinballBox::PinballBox() {
	top = Ground();
	down = Ground();
	left = Ground();
	rigth = Ground();
	x = 0;
	y = 0;
	h = 10;
	w = 10;

	/*	ground = Ground();
	ground.x = 0.0f; // [m]
	ground.y = 0.0f; // [m]
	ground.w = 30.0f; // [m]
	ground.h = 15.0f; // [m]*/
	/*this->x = x;
	this->y = y;
	this->w = w;
	this->h = h;

	top = Ground();
	top.x = x;
	top.y = y - h/2 + h*0.05f;
	top.w = w;
	top.h = h * 0.1f;


	down = Ground();
	down.x = x;
	down.y = y + h / 2 - h * 0.05f;
	down.w = w;
	down.h = h * 0.1f;


	left = Ground();
	left.x = x - w / 3;
	left.y = y;
	left.w = w / 2;
	left.h = h * 0.8f;

	rigth = Ground();
	rigth.x = x + w / 3;
	rigth.y = y;
	rigth.w = w / 2;
	rigth.h = h * 0.8f;*/

}



bool PinballBox::UpdateCollisions() {
	

	
	top.x = x - w/4;
	top.y = y + h - h*0.2f;
	top.w = w;
	top.h = h * 0.1f;




	
	down.x = x - w / 4;
	down.y = y- h*0.1f;
	down.w = w;
	down.h = h * 0.1f;


	
	left.x = x - w / 4;
	left.y = y;
	left.w = w / 2 ;
	left.h = h * 0.8f;

	
	rigth.x = x + w / 4;
	rigth.y = y;
	rigth.w = w / 2;
	rigth.h = h * 0.8f;

	

	return true;
}

//bool PinballBox::Draw() {
//	
//
//	return false;
//	
//
//
//
//}






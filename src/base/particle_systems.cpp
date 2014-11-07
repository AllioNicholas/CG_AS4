#include "particle_systems.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <numeric>

using namespace std;
using namespace FW;

namespace {

inline Vec3f fGravity(float mass) {
	return Vec3f(0, -9.8f * mass, 0);
}

// force acting on particle at pos1 due to spring attached to pos2 at the other end
inline Vec3f fSpring(const Vec3f& pos1, const Vec3f& pos2, float k, float rest_length) {
	// YOUR CODE HERE (R2)
	auto dist = pos1 - pos2;
	Vec3f springForce = (dist / dist.length()) * (rest_length - dist.length()) * k;
	return springForce;

}

inline Vec3f fDrag(const Vec3f& v, float k) {
	// YOUR CODE HERE (R2)
	Vec3f dragForce = -k * v;
	return dragForce;
}

} // namespace

void SimpleSystem::reset() {
	state_ = State(1, Vec3f(0, radius_, 0));
}

State SimpleSystem::evalF(const State& state) const {
	State f(1, Vec3f(-state[0].y, state[0].x, 0));
	return f;
}

Points SimpleSystem::getPoints() {
	return Points(1, state_[0]);
}

Lines SimpleSystem::getLines() {
	static const auto n_lines = 50u;
	auto l = Lines(n_lines * 2);
	const auto angle_incr = 2*FW_PI/n_lines;
	for (auto i = 0u; i < n_lines; ++i) {
		l[2*i] = l[2*i+1] =
			Vec3f(radius_ * FW::sin(angle_incr * i), radius_ * FW::cos(angle_incr * i), 0);
	}
	rotate(l.begin(), l.begin()+1, l.end());
	return l;
}

void SpringSystem::reset() {
	const auto start_pos = Vec3f(0.1f, -0.5f, 0.0f);
	const auto spring_k = 30.0f;
	state_ = State(4);
	// YOUR CODE HERE (R2)
	// Set the initial state for a particle system with one particle fixed
	// at origin and another particle hanging off the first one with a spring.
	// Place the second particle initially at start_pos.

	const auto start_part = Vec3f(0.0f, 0.0f, 0.0f);

	spring_.k = spring_k;
	spring_.rlen = start_pos.length();

	state_[0] = start_part;
	state_[1] = start_part; //start velocity
	state_[2] = start_pos;
	state_[3] = start_part; //start velocity

}

State SpringSystem::evalF(const State& state) const {
	const auto drag_k = 0.5f;
	const auto mass = 1.0f;
	State f(4);
	// YOUR CODE HERE (R2)
	// Return a derivative for the system as if it was in state "state".
	// You can use the fGravity, fDrag and fSpring helper functions for the forces.
	
	const auto start_part = Vec3f(0.0f, 0.0f, 0.0f);
	auto force = fGravity(mass) + fDrag(state[3], drag_k) + fSpring(state[2], state[0], spring_.k, spring_.rlen);

	//first particle stays fixed
	f[0] = start_part;
	f[1] = start_part;
	
	f[2] = state[3];
	f[3] = force / mass;

	return f;
}

Points SpringSystem::getPoints() {
	auto p = Points(2);
	p[0] = state_[0]; p[1] = state_[2];
	return p;
}

Lines SpringSystem::getLines() {
	auto l = Lines(2);
	l[0] = state_[0]; l[1] = state_[2];
	return l;
}

void PendulumSystem::reset() {
	const auto spring_k = 1000.0f;
	const auto start_point = Vec3f(0);
	const auto end_point = Vec3f(0.05, -1.5, 0);
	state_ = State(2*n_);
	// YOUR CODE HERE (R4)
	// Set the initial state for a pendulum system with n_ particles
	// connected with springs into a chain from start_point to end_point.
	
	state_[pos(0)] = start_point;

	springs_.clear(); //delete all previous data

	auto delta = (end_point - start_point) / (n_ - 1);

	for (auto i = 1; i < n_; i++) {
		Spring s = Spring(i, i - 1, spring_k, delta.length());
		springs_.push_back(s);

		state_[pos(i)] = state_[pos(i - 1)] + delta;
	}
	
}

unsigned PendulumSystem::vel(unsigned index) const {
	return (2 * index) + 1;
}

unsigned PendulumSystem::pos(unsigned index) const {
	return 2 * index;
}
  
State PendulumSystem::evalF(const State& state) const {
	const auto drag_k = 0.5f;
	const auto mass = 0.5f;
	auto f = State(2*n_);
	// YOUR CODE HERE (R4)
	// As in R2, return a derivative of the system state "state".

	//first particle stays fixed
	f[pos(0)] = state[vel(0)];
	f[vel(0)] = Vec3f(0.0f, 0.0f, 0.0f);
	
	//create, and initialize to 0 each element, vector where spring forces will be calculated taken into account both forces
	std::vector<Vec3f> sForce;
	for (auto i = 0; i < n_; i++)
		sForce.push_back(0);

	for (auto i = 0; i < n_ - 1; i++) {
		//calculationg forces taking into account both springs
		sForce[springs_[i].i1] += fSpring(state[pos(springs_[i].i1)], state[pos(springs_[i].i2)], springs_[i].k, springs_[i].rlen);
		sForce[springs_[i].i2] += fSpring(state[pos(springs_[i].i2)], state[pos(springs_[i].i1)], springs_[i].k, springs_[i].rlen);
	}
	
	for (auto i = 1; i < n_; i++) {
		Vec3f force = fGravity(mass) + fDrag(state[vel(i)], drag_k) + sForce[i];

		f[pos(i)] = state[vel(i)]; //derivate of position is velocity
		f[vel(i)] = force / mass; //derivate of velocity is acceleration
	}

	return f;
}

Points PendulumSystem::getPoints() {
	auto p = Points(n_);
	for (auto i = 0u; i < n_; ++i) {
		p[i] = state_[i*2];
	}
	return p;
}

Lines PendulumSystem::getLines() {
	auto l = Lines();
	for (const auto& s : springs_) {
		l.push_back(state_[2*s.i1]);
		l.push_back(state_[2*s.i2]);
	}
	return l;
}

void ClothSystem::reset() {
	const auto spring_k = 300.0f;
	const auto width = 1.5f, height = 1.5f; // width and height of the whole grid
	state_ = State(2*x_*y_);
	// YOUR CODE HERE (R5)
	// Construct a particle system with a x_ * y_ grid of particles,
	// connected with a variety of springs as described in the handout:
	// structural springs, shear springs and flex springs.

	auto length_x = width / (x_ - 1);
	auto length_y = height / (y_ - 1);
	auto length_diag = FW::sqrt(length_x*length_x + length_y*length_y);


}

State ClothSystem::evalF(const State& state) const {
	const auto drag_k = 0.08f;
	const auto n = x_ * y_;
	static const auto mass = 0.025f;
	auto f = State(2*n);
	// YOUR CODE HERE (R5)
	// This will be much like in R2 and R4.
	
	
	
	
	return f;
}

Points ClothSystem::getPoints() {
	auto n = x_ * y_;
	auto p = Points(n);
	for (auto i = 0u; i < n; ++i) {
		p[i] = state_[2*i];
	}
	return p;
}

Lines ClothSystem::getLines() {
	auto l = Lines();
	for (const auto& s : springs_) {
		l.push_back(state_[2*s.i1]);
		l.push_back(state_[2*s.i2]);
	}
	return l;
}

#pragma once

class ParticleSystem;

void eulerStep(ParticleSystem& ps, float step);

void trapezoidStep(ParticleSystem& ps, float step);

void midpointStep(ParticleSystem& ps, float step);

void rk4Step(ParticleSystem& ps, float step);

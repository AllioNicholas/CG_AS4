#include "integrators.hpp"

#include "utility.hpp"
#include "particle_systems.hpp"

void eulerStep(ParticleSystem& ps, float step) {
	// YOUR CODE HERE (R1)
	// Implement an Euler integrator.
	const auto& x0 = ps.state();
	auto n = x0.size();
	auto f0 = ps.evalF(x0);
	auto x1 = State(n);

	for (unsigned i = 0; i < n; i++) {
		x1[i] = x0[i] + step * f0[i];
	}

	ps.set_state(x1);

};

void trapezoidStep(ParticleSystem& ps, float step) {
	// YOUR CODE HERE (R3)
	// Implement a trapezoid integrator.

	const auto& x0 = ps.state();
	auto n = x0.size();
	auto f0 = ps.evalF(x0);
	auto x1 = State(n);
	auto xt = State(n);

	for (unsigned i = 0; i < n; i++) {
		x1[i] = x0[i] + step * f0[i];
	}

	auto ft = ps.evalF(x1);
	for (unsigned i = 0; i < n; i++) {
		xt[i] = x0[i] + (0.5f * step) * (f0[i] + ft[i]);
	}

	ps.set_state(xt);

}

void midpointStep(ParticleSystem& ps, float step) {
	const auto& x0 = ps.state();
	auto n = x0.size();
	auto f0 = ps.evalF(x0);
	auto xm = State(n), x1 = State(n);
	for (auto i = 0u; i < n; ++i) {
		xm[i] = x0[i] + (0.5f * step) * f0[i];
	}
	auto fm = ps.evalF(xm);
	for (auto i = 0u; i < n; ++i) {
		x1[i] = x0[i] + step * fm[i];
	}
	ps.set_state(x1);
}

void rk4Step(ParticleSystem& ps, float step) {
	// EXTRA: Implement the RK4 Runge-Kutta integrator.
}
 

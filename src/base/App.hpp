#pragma once

#include "particle_systems.hpp"

#include "gui/Window.hpp"
#include "gui/CommonControls.hpp"

#include <vector>

namespace FW {

struct Vertex
{
	Vec3f position;
	Vec3f normal;
};

struct glGeneratedIndices
{
	GLuint point_vao, mesh_vao;
	GLuint shader_program;
	GLuint vertex_buffer;
	GLuint model_to_world_uniform, world_to_clip_uniform;
};

class App : public Window::Listener
{
private:
	enum ParticleSystemType {
		SIMPLE_SYSTEM,
		SPRING_SYSTEM,
		PENDULUM_SYSTEM,
		CLOTH_SYSTEM
	};
	enum IntegratorType {
		EULER_INTEGRATOR,
		TRAPEZOID_INTEGRATOR,
		MIDPOINT_INTEGRATOR,
		RK4_INTEGRATOR
	};
public:
					App             (void);
	virtual bool    handleEvent     (const Window::Event& ev);

private:
	void			initRendering		(void);
	void			render				(void);

private:
					App             (const App&); // forbid copy
	App&            operator=       (const App&); // forbid assignment

private:
	Window			window_;
	CommonControls	common_ctrl_;

	bool			shading_toggle_;
	bool			shading_mode_changed_;
	bool			system_changed_;

	glGeneratedIndices	gl_;

	float				camera_rotation_angle_;
	Timer				timer_;

	ParticleSystemType	ps_type_;
	IntegratorType		integrator_;
	ParticleSystem*		ps_;
	
	float			step_;
	int				steps_per_update_;
	SimpleSystem	simple_system_;
	SpringSystem	spring_system_;
	PendulumSystem	pendulum_system_;
	ClothSystem		cloth_system_;
};

} // namespace FW

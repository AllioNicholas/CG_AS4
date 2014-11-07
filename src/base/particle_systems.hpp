#pragma once

#include "../framework/base/Math.hpp"

#include <vector>

typedef std::vector<FW::Vec3f> State;
typedef std::vector<FW::Vec3f> Points;
typedef std::vector<FW::Vec3f> Lines;

struct Spring {
	Spring() {}
	Spring(unsigned index1, unsigned index2, float spring_k, float rest_length) :
		i1(index1), i2(index2), k(spring_k), rlen(rest_length) {}
	unsigned i1, i2;
	float k, rlen;
};

class ParticleSystem {
public:
	virtual					~ParticleSystem() {};
	virtual State			evalF(const State&) const = 0;
	virtual void			reset() = 0;
	const State&			state() { return state_; }
	void					set_state(State s) { state_ = s; }
	virtual Points			getPoints() { return Points(); }
	virtual Lines			getLines() { return Lines(); }
protected:
	State					state_;
};

class SimpleSystem : public ParticleSystem {
public:
							SimpleSystem() : radius_(0.5f) { reset(); }
	State					evalF(const State&) const override;
	void					reset() override;
	Points					getPoints() override;
	Lines					getLines() override;

private:
	float					radius_;
};

class SpringSystem : public ParticleSystem {
public:
							SpringSystem() { reset(); }
	State					evalF(const State&) const override;
	void					reset() override;
	Points					getPoints() override;
	Lines					getLines() override;

private:
	Spring					spring_;
};

class PendulumSystem : public ParticleSystem {
public:
							PendulumSystem(unsigned n) : n_(n) { reset(); }
	State					evalF(const State&) const override;
	void					reset() override;
	Points					getPoints() override;
	Lines					getLines() override;
	unsigned				vel(unsigned index) const;
	unsigned				pos(unsigned index) const;

private:
	unsigned				n_;
	std::vector<Spring>		springs_;
};

class ClothSystem: public ParticleSystem {
public:
							ClothSystem(unsigned x, unsigned y) : x_(x), y_(y) { reset(); }
	State					evalF(const State&) const override;
	void					reset() override;
	Points					getPoints() override;
	Lines					getLines() override;
	FW::Vec2i				getSize() { return FW::Vec2i(x_, y_); }

private:
	unsigned				x_, y_;
	std::vector<Spring>		springs_;
};

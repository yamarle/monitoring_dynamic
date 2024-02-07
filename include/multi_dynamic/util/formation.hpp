#ifndef FORMATION_HPP
#define FORMATION_HPP

#include "dynamic.hpp"
#include "useless.hpp"

namespace formation_fmm{

	bool check_goal(Poss<int> agents, Poss<int> goal);
	bool in_range(Poss<int> agents, int ind, int x, int y, vector<vector<float>> crange);
	bool sec_range_inv(int x, int y, int ind, Poss<int> agents, int srange, int sx, int sy);
	void include_obstacles(vector<vector<float>> &grid, Poss<int> agents, Poss<int> obst, float vrange, int srange);
	void random_move(Poss<int> &agents, int ref, vector<vector<float>> crange, int srange, vector<vector<float>> grid);
	void select_move(Poss<int> &agents, int ref, vector<vector<float>> crange, int srange, vector<vector<float>> grid);
	void select_move(Poss<int> &agents, int ref, vector<vector<float>> crange, int srange, vector<vector<float>> grid, vector<vector<float>> grad);
	Poss<int> compute_subgoals(Path lpath, vector<vector<float>> grid, vector<vector<float>> crange, int srange);
	void iterate(Poss<int> agents, Poss<int> goal, Poss<int> obst, vector<vector<float>> grid, float vrange, vector<vector<float>> crange, int srange, float speed, string results_folder);

	// Lo mismo pero con gradientes en lugar de distancias
	vector<float> compute_ranges(vector<vector<float>> ranges);
	vector<vector<vector<float>>> range_gradients(Poss<int> pos, vector<vector<float>> grid, vector<float> crange);
	void security_range(vector<vector<float>> &grid, int x, int y, int srange);
	void security_range(vector<vector<float>> &grid, vector<int> x, vector<int> y, int srange);
	vector<vector<vector<float>>> range_gradients(Poss<int> pos, vector<vector<float>> grid, vector<float> crange, int srange);
	bool in_range(int x, int y, int ind, Poss<int> agents, vector<vector<float>> grid, vector<float> crange, float exp_dist);
	bool in_range(int ind, Poss<int> agents, vector<vector<float>> crange, vector<vector<vector<float>>> agents_grads);
	void move_agents(Poss<int> &agents, int ref, vector<vector<float>> grid, vector<vector<float>> crange, int srange, vector<float> ext_range, vector<vector<vector<float>>> &agents_grads);
	void move_agent(Poss<int> &agents, int ind, int ref, vector<vector<float>> grid, vector<vector<float>> crange, int srange, vector<float> ext_range, vector<vector<vector<float>>> &agents_grads);
	void iterate_gr(Poss<int> agents, Poss<int> goal, Poss<int> obst, vector<vector<float>> grid, float vrange, vector<vector<float>> crange, int srange, float speed, string results_folder);

}

#endif
// Calcula el plan global para desplegar una cadena, asigna objetivos a cada agente (con húngaro)
// Devuelve plan global (camino), vector de asignaciones, goals de los agentes, caminos de los agentes
// Todos del mismo tamaño -> nº agentes
// 
// Cositas:
// 	- Los objetivos de los agentes se posicionan sobre el camino directo al goal desde la base
// 	  Se tienen en cuenta los obstáculos. Se usa BRESENHAM
//	- No tiene en cuenta la comunicación entre los agentes en su camino a los goals


#ifndef CHAIN_PLAN_HPP
#define CHAIN_PLAN_HPP

#include "fmm_2.hpp"
#include "path.hpp"
#include "hungarian.hpp"
#include "bresenham.hpp"

class chain_plan{
private:

	int n, n_; // Número de agentes total y los que se van a utilizar

	vector<vector<float>> grid; // grid
	vector<vector<float>> grad; // gradiente

	Poss<int> ag_pos, goal_pos;

	Path global_plan;
	vector<Path> paths;
	vector<int> alloc;

	int err=0;

public:
	chain_plan();
	void solve(int xi, int yi, int xg, int yg, vector<vector<float>> grid, Poss<int> pos, float crange);

	void check(Path global_, vector<vector<float>> grid_, float range);

	Poss<int> goals(float range);
	Poss<int> goals_(float range);
	void compute_paths(float range);
	// Devolver
	Path get_plan();
	vector<Path> get_paths();
	Poss<int> get_pos();
	vector<int> get_alloc();
	int get_error();
};

#endif
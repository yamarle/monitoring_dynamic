#include <multi_dynamic/util/chain_plan.hpp>

chain_plan::chain_plan(){}

void chain_plan::solve(int xi, int yi, int xg, int yg, vector<vector<float>> grid, Poss<int> pos, float range)
{
	if(n){
		paths.clear();
		goal_pos.clear();
		global_plan.clear();
		alloc.clear();
	}

	n=pos.x.size();
	paths.resize(n);
	goal_pos(n);
	this->ag_pos=pos;
	this->grid=grid;
	// Plan global (camino)
	FMM gr(xi,yi,grid);
	grad=gr.compute_gradient_();
	global_plan.gradient_descent_(grad,xg,yg);
	if(!global_plan.tam){
		err=1; // Atrapado
		return;
	}
	global_plan.compute_distance();
	if(global_plan.d>range*n){
		global_plan.clear();
		err=2; // No hay suficientes robots
		return;
	}
	// Asignar y calcular los caminos
	compute_paths(range);
}

void chain_plan::check(Path global_, vector<vector<float>> grid_, float range)
{
	paths.clear();
	goal_pos.clear();
	alloc.clear();

	global_plan=global_;

	if(!global_plan.tam){
		err=1; // Atrapado
		return;
	}

	global_plan.compute_distance();
	if(global_plan.d>range*n){
		global_plan.clear();
		err=2; // No hay suficientes robots
		return;
	}

	paths.resize(n);
	goal_pos(n);

	// Asignar y calcular los caminos
	compute_paths(range);
}

Poss<int> chain_plan::goals(float range)
{
	// Aquí es donde se obtienen las posiciones de los relays sobre la cadena
	Poss<int> res;
	n_=n-1;
	float dacum=0;
	res.x.push_back(global_plan.x[global_plan.tam-1]);
	res.y.push_back(global_plan.y[global_plan.tam-1]);
	Poss<int> los_pos;
	int k;
	// Propagar dentro del radio de cobertura
	FMM gr(res.x[0],res.y[0],grid);
	grad = gr.expand_dist(range,los_pos);
	// Eliminar los puntos que están obstruidos por una pared
	k=0;
	while(k<los_pos.x.size()){
	    if(bresenham::exists_wall(grid, res.x[0], res.y[0], los_pos.x[k], los_pos.y[k])){
	    	grad[los_pos.x[k]][los_pos.y[k]]=INF;
	    	los_pos.erase(k);
	    }else k++;
	}
	los_pos.clear();
	for(int i=global_plan.tam-2; i>=0; i--){
		if(grad[global_plan.x[i]][global_plan.y[i]]==INF){
			// Me estoy saliendo del área de cobertura
			n_--;
			res.x.insert(res.x.begin(),global_plan.x[i+1]);
			res.y.insert(res.y.begin(),global_plan.y[i+1]);
			if(!n_) break;
			// Propagar dentro del radio de cobertura
			FMM gr(res.x[0],res.y[0],grid);
			grad = gr.expand_dist(range,los_pos);
			// Eliminar los puntos que están obstruidos por una pared
			k=0;
			while(k<los_pos.x.size()){
			    if(bresenham::exists_wall(grid, res.x[0], res.y[0], los_pos.x[k], los_pos.y[k])){
			    	grad[los_pos.x[k]][los_pos.y[k]]=INF;
			    	los_pos.erase(k);
			    }else k++;
			}
			// Vaciar las posiciones visibles
			los_pos.clear();
		}
	}

	// El último gradiente para la distancia a la base
	FMM gr_(res.x[0],res.y[0],grid);
	grad = gr_.expand_dist(range,los_pos);
	k=0;
	while(k<los_pos.x.size()){
	    if(bresenham::exists_wall(grid, res.x[0], res.y[0], los_pos.x[k], los_pos.y[k])){
	    	grad[los_pos.x[k]][los_pos.y[k]]=INF;
	    	los_pos.erase(k);
	    }else k++;
	}
	if(grad[global_plan.x[0]][global_plan.y[0]]==INF) err=3;

	n_=n-n_;
	return res;
}

Poss<int> chain_plan::goals_(float range)
{
	// Aquí es donde se obtienen las posiciones de los relays sobre la cadena
	Poss<int> res;
	n_=n-1;
	float dacum=0;
	res.x.push_back(global_plan.x[global_plan.tam-1]);
	res.y.push_back(global_plan.y[global_plan.tam-1]);
	for(int i=global_plan.tam-2; i; i--){
		dacum+=global_plan.dpp[i];
		if(dacum>=range){
			dacum=0;
			n_--;
			res.x.insert(res.x.begin(),global_plan.x[i+1]);
			res.y.insert(res.y.begin(),global_plan.y[i+1]);
			if(!n_) break;
		}
	}
	n_=n-n_;
	return res;
}

void chain_plan::compute_paths(float range)
{
	// Obtener los goals
	Poss<int> goals_=goals(range);
	vector<vector<float>> costs(n,vector<float>(n_,INF));
	vector<vector<Path>> p(n,vector<Path>(n_));
	for(int i=0; i<n; i++){
		FMM gr(ag_pos.x[i],ag_pos.y[i],grid);
		grad=gr.compute_gradient_();
		for(int j=0; j<n_; j++){
			costs[i][j]=grad[goals_.x[j]][goals_.y[j]];
			p[i][j].gradient_descent_(grad,goals_.x[j],goals_.y[j]);
		}
	}
	hungarian a;
	a.solve(costs,alloc);
	for(int i=0; i<n; i++){
		if(alloc[i]>=0){
			paths[i] = p[i][alloc[i]];
			// Guardar bien la última ag_posición (La última del camino)
			goal_pos.x[i]=p[i][alloc[i]].x[p[i][alloc[i]].tam-1];
			goal_pos.y[i]=p[i][alloc[i]].y[p[i][alloc[i]].tam-1];
		}else{
			goal_pos.x[i]=0; goal_pos.y[i]=0;
			paths[i].set(0,0);
		}
	}
}

Path chain_plan::get_plan()
{
	return global_plan;
}

vector<Path> chain_plan::get_paths()
{
	return paths;
}

Poss<int> chain_plan::get_pos()
{
	return goal_pos;
}

vector<int> chain_plan::get_alloc()
{
	return alloc;
}

int chain_plan::get_error()
{
	// 0: Todo OK
	// 1: Atrapado
	// 2: No hay suficientes robots (d_short>nrobots*crange)
	// 3: No hay suficientes robots (Oclusión por los obstáculos)
	return err;
}
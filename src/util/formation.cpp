
#include <multi_dynamic/util/formation.hpp>

bool formation_fmm::check_goal(Poss<int> agents, Poss<int> goal)
{
	for(int i=0; i<agents.x.size(); i++)
		if(agents.x[i]==goal.x[0] && agents.y[i]==goal.y[0])
			return true;
	return false;
}

bool formation_fmm::in_range(Poss<int> agents, int ind, int x, int y, vector<vector<float>> crange)
{
	for(int i=0; i<agents.x.size(); i++){
		if(i!=ind && sqrt(pow(agents.x[i]-x,2)+pow(agents.y[i]-y,2))>crange[i][ind]){
			return false;
		}
	}
	return true;
}

bool formation_fmm::sec_range_inv(int x, int y, int ind, Poss<int> agents, int srange, int sx, int sy)
{
	
	for(int i=x-srange; i<x+srange; i++){
		for(int j=y-srange; j<y+srange; j++){
			if(i>=0 && i<sx && j>=0 && j<sy){
				for(int k=0; k<agents.x.size(); k++){
					if(i==agents.x[k] && j==agents.y[k]){
						return true;
					}
				}
			}
		}
	}

	/*
	for(int i=0; i<agents.x.size(); i++){
		if(i!=ind && sqrt(pow(agents.x[i]-x,2)+pow(agents.y[i]-y,2))<=srange){
			return false;
		}
	}
	*/

	return false;
}

void formation_fmm::random_move(Poss<int> &agents, int ref, vector<vector<float>> crange, int srange, vector<vector<float>> grid)
{
	int random_move;
	int x, y;
	for(int i=0; i<agents.x.size(); i++){
		if(i==ref) continue;
		// Generar un movimiento aleatorio
		random_move = static_cast<float>(rand())/static_cast<float>(RAND_MAX/9);
		x = agents.x[i] + dynamic_fmm::move_x[random_move];
		y = agents.y[i] + dynamic_fmm::move_y[random_move];
		while(!grid[x][y] || !in_range(agents, i, x, y, crange) || sec_range_inv(x, y, i, agents, srange, grid.size(), grid[0].size())){
			cout<<grid[x][y]<<" - "<<in_range(agents, i, x, y, crange)<<" - "<<sec_range_inv(x, y, i, agents, srange, grid.size(), grid[0].size())<<endl;
			random_move = static_cast<float>(rand())/static_cast<float>(RAND_MAX/9);
			x = agents.x[i] + dynamic_fmm::move_x[random_move];
			y = agents.y[i] + dynamic_fmm::move_y[random_move];
			cin.get();
		}
		agents.x[i] = x; agents.y[i] = y;
	}
}

void formation_fmm::select_move(Poss<int> &agents, int ref, vector<vector<float>> crange, int srange, vector<vector<float>> grid)
{
	int random_move;
	int x, y, ox, oy;
	int om;
	float dist, odist;
	for(int i=0; i<agents.x.size(); i++){
		if(i==ref) continue;
		// Cuál es el mejor movimiento
		dist = odist = INF;
		for(int j=0; j<dynamic_fmm::move_x.size(); j++){
			x = agents.x[i] + dynamic_fmm::move_x[j];
			y = agents.y[i] + dynamic_fmm::move_y[j];
			// Espacio libre & dentro del rango que preestablecido & sin invadir la distancia de seguridad
			if(grid[x][y] && in_range(agents, i, x, y, crange) && !sec_range_inv(x, y, i, agents, srange, grid.size(), grid[0].size())){
				// Mejoro la distancia al lider?
				dist = sqrt(pow(agents.x[ref]-x,2)+pow(agents.y[ref]-y,2));
				if(odist > dist){
					odist = dist;
					ox = x; oy = y;
					om = j;
				}
			}
		}
		// Mover a la posición óptima
		if(odist < INF){
			agents.x[i] = ox;
			agents.y[i] = oy;
		}
	}
}

void formation_fmm::select_move(Poss<int> &agents, int ref, vector<vector<float>> crange, int srange, vector<vector<float>> grid, vector<vector<float>> grad)
{
	int random_move;
	int x, y, ox, oy;
	int om;
	float dist, odist;
	for(int i=0; i<agents.x.size(); i++){
		if(i==ref) continue;
		// Cuál es el mejor movimiento
		dist = odist = INF;
		for(int j=0; j<dynamic_fmm::move_x.size(); j++){
			x = agents.x[i] + dynamic_fmm::move_x[j];
			y = agents.y[i] + dynamic_fmm::move_y[j];
			// Espacio libre & dentro del rango que preestablecido & sin invadir la distancia de seguridad
			if(grad[x][y]<INF) // Dentro del rango del lider
			if(grid[x][y] && in_range(agents, i, x, y, crange) && !sec_range_inv(x, y, i, agents, srange, grid.size(), grid[0].size())){
				// Mejoro la distancia al lider?
				if(odist > grad[x][y]){
					odist = grad[x][y];
					ox = x; oy = y;
					om = j;
				}
			}
		}
		// Mover a la posición óptima
		if(odist < INF){
			agents.x[i] = ox;
			agents.y[i] = oy;
		}
	}
	cout<<"Dist move: ";
	for(int i=0; i<agents.x.size(); i++){
		cout<<grad[agents.x[i]][agents.y[i]]<<" ";
	}
	cout<<endl;
}

void formation_fmm::iterate(Poss<int> agents, Poss<int> goal, Poss<int> dyn_obst, vector<vector<float>> grid, float vrange, vector<vector<float>> crange, int srange, float speed, string results_folder)
{
	int Nagents = agents.x.size();
	vector<Path> paths(Nagents);

	vector<vector<float>> team_grid = grid; // Los obstáculos que ve el equipo entero
	vector<vector<float>> real_grid = grid; // El grid real, con los obstáculos del entorno (MUNDO REAL)

	int Nobst = dyn_obst.x.size();

	vector<vector<float>> goal_grad; // Gradiente del objetivo
	vector<vector<float>> aux_grid; // Auxiliar, para ir buscando otros caminos

	vector<vector<float>> grad; // Gradiente

	int leader = 0, pt;
	float val = 0;

	int x_, y_;
	Poss<int> aux_pos;

	float mcr = 0;
	vector<vector<float>> distance;

	int cont = 0;
	while(!check_goal(agents, goal)){
		// GUARDAR LAS POSICIONES DE LOS OBSTÁCULOS Y DEL AGENTE
        agents.save((results_folder+"agents_"+to_string(cont)+".txt").c_str()); // Los agents
        dyn_obst.save((results_folder+"dyn_obst_"+to_string(cont)+".txt").c_str()); // Los obstaculos

        // Fijar los obstáculos dinámicos
        real_grid = grid;
        for(int i=0; i<Nobst; i++){
        	real_grid[dyn_obst.x[i]][dyn_obst.y[i]];
        }

        // Mover los obstáculos de manera aleatoria
        dynamic_fmm::random_movement(dyn_obst, real_grid);

        // Meter los obstáculos en el grid visible por el equipo
        team_grid = grid;
        for(int i=0; i<Nagents; i++)
        	dynamic_fmm::include_obstacles(team_grid, dyn_obst, agents.x[i], agents.y[i], vrange, srange);

        // 1. Calcular el gradiente al goal
        FMM ggr(goal.x[0], goal.y[0], team_grid);
        goal_grad = ggr.compute_gradient_();
        // 2. Seleccionar al lider (En base a la mínima distancia)
        val = INF;
        for(int i=0; i<Nagents; i++){
        	if(val>goal_grad[agents.x[i]][agents.y[i]]){
        		val = goal_grad[agents.x[i]][agents.y[i]];
        		leader = i;
        	}
        }

        // 3. Sacar el camino del lider
        paths[leader].gradient_descent(goal_grad,agents.x[leader],agents.y[leader]);
        paths[leader].compute_time(speed);

        // 4. Comprobar si se puede mover al lider

        // Sacar el rango máximo
        mcr = 0;
        for(int i=0; i<Nagents; i++)
			if(crange[leader][i]<INF && mcr<crange[leader][i])
				mcr = crange[leader][i];
		// Expandir el gradiente
        FMM lgr(agents.x[leader], agents.y[leader], team_grid);
        grad = lgr.expand_dist(mcr);

        if(paths[leader].tam>1){
        	/*
        	x_ = paths[leader].x[1]; y_ = paths[leader].y[1];
        	// Comprobar si se sale del rango preestablecido
        	if(in_range(agents, leader, x_, y_, crange)){
        		agents.x[leader] = x_; agents.y[leader] = y_;
        	}
        	*/
        	mcr = 0;
        	cout<<"Dist lead: ";
        	for(int i=0; i<Nagents; i++){
        		if(mcr<grad[agents.x[i]][agents.y[i]]){
        			mcr = grad[agents.x[i]][agents.y[i]];
        		}
        		cout<<grad[agents.x[i]][agents.y[i]]<<" ";
        	}
        	cout<<endl;
        	if(mcr<INF){
        		agents.x[leader] = paths[leader].x[1]; agents.y[leader] = paths[leader].y[1];
        		cout<<"MUEVO AL LIDER"<<endl;
        	}
        }

        // 5. Calcular los movimientos de los followers
        //random_move(agents, pt, crange, srange, team_grid);
        //select_move(agents, leader, crange, srange, team_grid);
        select_move(agents, leader, crange, srange, team_grid, grad);
        distance = distance_matrix(agents);

        cout<<"Lider: "<<leader<<endl;
        agents.show_();

        cout<<"Distancias"<<endl;
        sh_matr_al(distance);

        cin.get();

        cont++;

	}

}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Con gradientes
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

vector<float> formation_fmm::compute_ranges(vector<vector<float>> ranges)
{
	vector<float> res(ranges.size(),0);	
	for(int i=0; i<ranges.size(); i++)
		for(int j=0; j<ranges.size(); j++)
			if(ranges[i][j]<INF && res[i]<ranges[i][j])
				res[i]<ranges[i][j];
	return res;
}

vector<vector<vector<float>>> formation_fmm::range_gradients(Poss<int> pos, vector<vector<float>> grid, vector<float> crange)
{
	vector<vector<vector<float>>> res(pos.x.size());
	for(int i=0; i<pos.x.size(); i++){
		FMM gr(pos.x[i], pos.y[i], grid);
		res[i] = gr.expand_dist(crange[i]);
	}
	return res;
}

void formation_fmm::security_range(vector<vector<float>> &grid, int x, int y, int srange)
{
	for(int i=x-srange; i<x+srange; i++)
		for(int j=y-srange; j<y+srange; j++)
			if(i>=0 && i<grid.size() && j>=0 && j<grid[0].size())
				grid[i][j] = 0;
}

void formation_fmm::security_range(vector<vector<float>> &grid, vector<int> x, vector<int> y, int srange)
{
	for(int cc = 0; cc<x.size(); cc++)
	for(int i=x[cc]-srange; i<x[cc]+srange; i++)
		for(int j=y[cc]-srange; j<y[cc]+srange; j++)
			if(i>=0 && i<grid.size() && j>=0 && j<grid[0].size())
				grid[i][j] = 0;
}

vector<vector<vector<float>>> formation_fmm::range_gradients(Poss<int> pos, vector<vector<float>> grid, vector<float> crange, int srange)
{
	vector<vector<vector<float>>> res(pos.x.size());

	// Rango de seguridad en el grid
	vector<vector<float>> original_grid = grid;
	for(int i=0; i<pos.x.size(); i++){
		for(int j=0; j<pos.x.size(); j++) // De todos los agentes menos "yo"
			if(j!=i) security_range(grid, pos.x[j], pos.y[j], srange);

		FMM gr(pos.x[i], pos.y[i], grid);
		res[i] = gr.expand_dist(crange[i]);
		grid = original_grid;
	}
	return res;
}

bool formation_fmm::in_range(int x, int y, int ind, Poss<int> agents, vector<vector<float>> grid, vector<float> crange, float exp_dist)
{
	FMM gr(x,y,grid);
	vector<vector<float>> grad = gr.expand_dist(exp_dist);
	for(int i=0; i<agents.x.size(); i++){
		if(grad[agents.x[i]][agents.y[i]]==INF && grad[agents.x[i]][agents.y[i]]<crange[i]){
			return false;
		}
	}
	return true;
}

bool formation_fmm::in_range(int ind, Poss<int> agents, vector<vector<float>> crange, vector<vector<vector<float>>> agents_grads)
{
	for(int i=0; i<agents.x.size(); i++){
		if(i!=ind && agents_grads[ind][agents.x[i]][agents.y[i]]==INF && agents_grads[ind][agents.x[i]][agents.y[i]]<crange[ind][i]){
			return false;
		}
	}
	return true;
}

void formation_fmm::move_agents(Poss<int> &agents, int ref, vector<vector<float>> grid, vector<vector<float>> crange, int srange, vector<float> ext_range, vector<vector<vector<float>>> &agents_grads)
{
	int x, y, ox, oy, sx, sy, om;
	float odist;
	sx = grid.size(); sy = grid[0].size();
	for(int i=0; i<agents.x.size(); i++){
		if(i!=ref){
			// Seleccionar el mejor movimiento
			odist = INF;
			for(int j=0; j<dynamic_fmm::move_x.size(); j++){
				x = agents.x[i] + dynamic_fmm::move_x[j];
				y = agents.y[i] + dynamic_fmm::move_y[j];
				if(x>=0 && x<sx && y>=0 && y<sy) // dentro del grid
				if(grid[x][y] && in_range(i, agents, crange, agents_grads)){ // No es un obstáculo
					if(odist > agents_grads[ref][x][y]){
						odist = agents_grads[ref][x][y];
						ox = x; oy = y;
						om = j;
					}
				}
			}
			
			if(odist<INF){
				// Mover
				agents.x[i] = ox;
				agents.y[i] = oy;
				// Actualizar el gradiente del agente
				FMM gr(agents.x[i], agents.y[i], grid);
				agents_grads[i] = gr.expand_dist(ext_range[i]);
			}

		}
	}
}

void formation_fmm::move_agent(Poss<int> &agents, int ind, int ref, vector<vector<float>> grid, vector<vector<float>> crange, int srange, vector<float> ext_range, vector<vector<vector<float>>> &agents_grads)
{
	// grid: contiene los obstáculos dinámicos, pero NO a los compañeros
	int x, y, ox, oy, sx, sy, om;
	float odist;
	sx = grid.size(); sy = grid[0].size();

	// Seleccionar el mejor movimiento (El más cercano al lider)
	odist = INF;
	for(int j=0; j<dynamic_fmm::move_x.size(); j++){
		x = agents.x[ind] + dynamic_fmm::move_x[j];
		y = agents.y[ind] + dynamic_fmm::move_y[j];
		if(x>=0 && x<sx && y>=0 && y<sy) // dentro del grid
		if(grid[x][y] && in_range(ind, agents, crange, agents_grads)){ // No es un obstáculo
			if(odist > agents_grads[ref][x][y]){
				odist = agents_grads[ref][x][y];
				ox = x; oy = y;
				om = j;
			}
		}
	}
	
	if(odist<INF){
		// Mover
		agents.x[ind] = ox;
		agents.y[ind] = oy;
		// Actualizar el gradiente del agente
		FMM gr(agents.x[ind], agents.y[ind], grid);
		agents_grads[ind] = gr.expand_dist(ext_range[ind]);
	}

}

void formation_fmm::iterate_gr(Poss<int> agents, Poss<int> goal, Poss<int> dyn_obst, vector<vector<float>> grid, float vrange, vector<vector<float>> crange, int srange, float speed, string results_folder)
{
	int Nagents = agents.x.size();
	vector<Path> paths(Nagents);

	vector<vector<float>> team_grid = grid; // Los obstáculos que ve el equipo entero
	vector<vector<float>> real_grid = grid; // El grid real, con los obstáculos del entorno (MUNDO REAL)

    vector<vector<float>> obstacles_grid;

	int Nobst = dyn_obst.x.size();

	vector<vector<float>> goal_grad; // Gradiente del objetivo
	vector<vector<float>> aux_grid; // Auxiliar, para ir buscando otros caminos

	vector<vector<float>> grad; // Gradiente

	int leader = 0, pt;
	float val = 0;

	int x_, y_;
	Poss<int> aux_pos;

	// Distancias de expanción máxima
	vector<float> mcr(Nagents,0);
	for(int i=0; i<Nagents; i++){
		for(int j=0; j<Nagents; j++){
			if(crange[i][j]<INF && mcr[i]<crange[i][j])
				mcr[i] = crange[i][j];
		}
	}

	// Gradientes en los que se almacena la distancia de los agentes
	vector<vector<vector<float>>> agents_grads(Nagents);

	bool ch_range;

	int cont = 0;
	while(!check_goal(agents, goal)){
		// GUARDAR LAS POSICIONES DE LOS OBSTÁCULOS Y DEL AGENTE
        agents.save((results_folder+"agents_"+to_string(cont)+".txt").c_str()); // Los agents
        dyn_obst.save((results_folder+"dyn_obst_"+to_string(cont)+".txt").c_str()); // Los obstaculos

        // Fijar los obstáculos dinámicos
        real_grid = grid;
        for(int i=0; i<Nobst; i++){
        	real_grid[dyn_obst.x[i]][dyn_obst.y[i]];
        }

        // Mover los obstáculos de manera aleatoria
        dynamic_fmm::random_movement(dyn_obst, real_grid);

        // Meter los obstáculos en el grid visible por el equipo
        team_grid = grid;
        for(int i=0; i<Nagents; i++)
        	dynamic_fmm::include_obstacles(team_grid, dyn_obst, agents.x[i], agents.y[i], vrange, srange);

        // Almacenar el grid con los obstáculos
        obstacles_grid = team_grid;

        // 1. Calcular el gradiente al goal
        FMM ggr(goal.x[0], goal.y[0], team_grid);
        goal_grad = ggr.compute_gradient_();
        // 2. Seleccionar al lider (En base a la mínima distancia)
        val = INF;
        for(int i=0; i<Nagents; i++){
        	if(val>goal_grad[agents.x[i]][agents.y[i]]){
        		val = goal_grad[agents.x[i]][agents.y[i]];
        		leader = i;
        	}
        }

        // 3. Sacar el camino del lider
        paths[leader].gradient_descent(goal_grad,agents.x[leader],agents.y[leader]);
        paths[leader].compute_time(speed);

        // 4. Comprobar si se puede mover al lider
		// Expandir los gradientes
		cout<<"VOY A PROPAGAR"<<endl;
		agents_grads = range_gradients(agents, team_grid, mcr, srange);

        if(paths[leader].tam>1){
        	ch_range = true;
        	for(int i=0; i<Nagents; i++){
        		// No es un obstáculo & está fuera de rango de señal
        		//out<<"1: "<<agents_grads[leader][agents.x[i]][agents.y[i]]<<endl;
        		//cout<<"2("<<leader<<", "<<i<<"/"<<Nagents<<"): "<<crange[leader][i]<<endl;
        		//if(agents_grads[leader][agents.x[i]][agents.y[i]]<INF && crange[leader][i]<agents_grads[leader][agents.x[i]][agents.y[i]]){
    			if(crange[leader][i]<agents_grads[leader][agents.x[i]][agents.y[i]]){
        			ch_range = false;
        			break;
        		}
        	}
        	if(ch_range){ // Muevo al líder
        		agents.x[leader] = paths[leader].x[1]; agents.y[leader] = paths[leader].y[1];
        	}
        }

        // 5. Calcular los movimientos de los followers
        //move_agents(agents, leader, team_grid, crange, srange, mcr, agents_grads);
        for(int i=0; i<Nagents; i++){
        	if(i!=leader){
        		//cout<<i<<": "<<agents_grads[leader][agents.x[i]][agents.y[i]]<<endl;
        		move_agent(agents, i, leader, team_grid, crange, srange, mcr, agents_grads);
        		team_grid = obstacles_grid;
        		dynamic_fmm::include_obstacles(team_grid, dyn_obst, agents.x[i], agents.y[i], vrange, srange);
        	}
        }

        cout<<"Lider: "<<leader<<endl;
        agents.show_();

        cin.get();

        cont++;

	}

}
#ifndef ALLOCATIONS_HPP
#define ALLOCATIONS_HPP

#include "fmm_2.hpp"
#include "path.hpp"
#include <limits>
#include <algorithm>
#include <ctime>

namespace allocation{

	int random_seed(int n){
		return rand()%n;
	} // semilla aleatoria para generación de combinaciones

	//-------------------------------------------------------------------------------------
	// 									UTILIDADES
	//-------------------------------------------------------------------------------------	
	vector<vector<vector<float>>> compute_all_gradients(Poss<int> pos, vector<vector<float>> grid)
	{
		// pos: la primera posición es la de origen
		vector<vector<vector<float>>> res(pos.x.size());
		for(int i=0; i<pos.x.size(); i++){
			FMM gr(pos.x[i],pos.y[i],grid);
			res[i] = gr.compute_gradient_();
		}
		return res;
	}

	vector<vector<vector<float>>> compute_all_gradients(Poss<int> ini, Poss<int> pos, vector<vector<float>> grid)
	{
		pos.x.insert(pos.x.begin(),ini.x[0]);
		pos.y.insert(pos.y.begin(),ini.y[0]);
		vector<vector<vector<float>>> res = compute_all_gradients(pos, grid);
		return res;
	} // Lo mismo de antes, pero la posición de inicio no está dentro de "pos"

	vector<vector<float>> form_cost_matrix(Poss<int> pos, vector<vector<vector<float>>> grads, float speed)
	{
		vector<vector<float>> res(pos.x.size(), vector<float>(pos.x.size(),0));
		for(int i=0; i<pos.x.size(); i++)
			for(int j=i+1; j<pos.x.size(); j++){
				res[i][j]=grads[i][pos.x[j]][pos.y[j]]/speed;
				res[j][i] = res[i][j];
			}
		return res;
	}

	float simple_tour_cost(vector<int> tour, vector<vector<float>> costs)
	{
		float res=0;
		for(int i=0; i<tour.size()-1; i++){
			res+=costs[i][i+1];
		}
		return res;
	}

	// Cálculo de los caminos reales a traves de los puntos de la ruta calculada

	Path compute_path(Poss<int> pos, vector<int> tour, vector<vector<float>> grid)
	{
		Path res, aux;

		for(int i=0; i<tour.size()-1; i++){
			if(tour[i+1]>0){
				FMM gr(pos.x[tour[i]],pos.y[tour[i]],grid);
				aux.gradient_descent_(gr.compute_gradient_(),pos.x[tour[i+1]],pos.y[tour[i+1]]);
				res.append_pos(aux);
				aux.clear();
			}
		}

		return res;
	} // Se calculan los gradientes dentro (Sólo calcula posiciones)

	Path compute_path(Poss<int> pos, vector<int> tour, vector<vector<vector<float>>> grads)
	{
		Path res, aux;

		for(int i=0; i<tour.size()-1; i++){
			if(tour[i+1]>0){
				aux.gradient_descent_(grads[tour[i]],pos.x[tour[i+1]],pos.y[tour[i+1]]);
				res.append_pos(aux);
				aux.clear();
			}
		}

		return res;
	} // Recibe los gradientes ya calculados (Sólo calcula posiciones)

	Path compute_path(Poss<int> pos, vector<int> tour, vector<vector<vector<float>>> grads, float working_time, float speed)
	{
		Path res, aux;

		res.initialize(pos.x[0],pos.y[0]);

		for(int i=0; i<tour.size()-1; i++){
			if(tour[i+1]>0){
				aux.gradient_descent_(grads[tour[i]],pos.x[tour[i+1]],pos.y[tour[i+1]]);
				aux.compute_time(speed);
				res.append_(aux);
				res.insert_wait(working_time);
				aux.clear();
			}
		}

		return res;
	} // Recibe los gradientes ya calculados y los tiempos de espera (Tambien calcula el tiempo del recorrido)

	Path compute_path_wg(Poss<int> pos, vector<int> tour, vector<vector<vector<float>>> grads, float working_time, float speed, float global_path_time, vector<float> &goal_times)
	{
		Path res, aux;

		/*
		if(goal_times.size()!=pos.x.size()-1){
			goal_times.resize(pos.x.size());
		}
		*/

		if(goal_times.size()){
			goal_times.clear();
		}

		res.initialize(pos.x[0],pos.y[0]);

		for(int i=0; i<tour.size()-1; i++){
			if(tour[i+1]>0){
				aux.gradient_descent_(grads[tour[i]],pos.x[tour[i+1]],pos.y[tour[i+1]]);
				aux.compute_time(speed);
				res.append_(aux);
				res.insert_wait(working_time);
				//goal_times[tour[i+1]] = res.t + global_path_time;
				goal_times.push_back(res.t + global_path_time);
				aux.clear();
			}
		}

		return res;
	} // Recibe los gradientes ya calculados y los tiempos de espera, devuelve el tiempo cuando se han realizado los trabajos

	vector<float> reach_critical_point(int x, int y, Poss<int> pos, vector<vector<float>> grid, float speed)
	{
		vector<float> res(pos.x.size(),0);

		FMM gr(x,y,grid);
		vector<vector<float>> grad = gr.compute_gradient_();

		for(int i=0; i<pos.x.size(); i++){
			res[i] = grad[pos.x[i]][pos.y[i]]/speed;
		}

		return res;
	} // Lo que se tarda en alcanzar el punto del colector

	vector<float> reach_critical_point(int x, int y, vector<vector<vector<float>>> grads, float speed)
	{
		vector<float> res(grads.size(),0);

		for(int i=0; i<grads.size(); i++){
			res[i] = grads[i][x][y]/speed;
		}

		return res;
	} // Lo que se tarda en alcanzar el punto del colector

	//================================================================================================
	// 							ASIGNACIONES
	//================================================================================================
	// !!!!!!!!!!!!!!!!!TODAS LAS FUNCIONES DEVUELVEN LA COMBINACIÓN!!!!!!!!!!!!!!!!!!!!!!!
	// !!!!!!!!!!!!!!!!!LA PRIMERA FILA/COLUMNA ES EL PUNTO DE PARTIDA!!!!!!!!!!!!!!!!!!!!!


	//-------------------------------------------------------------------------------------
	// SIMPLES

	vector<int> nn(vector<vector<float>> costs) // Nearest Neighbor
	{
		int n = costs.size();
		vector<int> res(n,0);

		int ind = 0, ind_; // El punto de partida
		float min;
		vector<bool> td(n,true);
		for(int i=1; i<n; i++){
			min = numeric_limits<float>::max();
			ind_=0;
			for(int j=1; j<n; j++){
				if(td[j] && min>costs[ind][j]){
					ind_ = j;
					min = costs[ind][j];
				}
			}
			if(ind_){
				res[i] = ind_;
				ind = ind_;
				td[ind] = false;
			}
		}

		return res;
	}

	vector<int> nn_rs(vector<vector<float>> costs, float time) // Nearest Neighbor + Random Shuffle
	{
		vector<int> curr_tour = nn(costs);
		float curr_cost = simple_tour_cost(curr_tour, costs);

		vector<int> best_tour = curr_tour;
		float best_cost = curr_cost;

		float t = 0;
		clock_t dt;
		while(t<time){
			dt = clock();
			random_shuffle(curr_tour.begin()+1,curr_tour.end(),random_seed);
			curr_cost = simple_tour_cost(curr_tour, costs);
			if(curr_cost<best_cost){
				best_tour = curr_tour;
				best_cost = curr_cost;
			}
			dt = clock() - dt;
			t+=(float)dt/CLOCKS_PER_SEC;
		}

		return best_tour;
	}


	vector<int> nn_2o(vector<vector<float>> costs) // Nearest Neighbor + 2-opt
	{
		vector<int> curr_tour = nn(costs); // Combinación NN
		//for(int i=0; i<curr_tour.size(); i++) curr_tour[i] = i; // Por poner algo
		float curr_cost = simple_tour_cost(curr_tour, costs);

		vector<int> best_tour = curr_tour;
		float best_cost = curr_cost;

		int n = costs.size(), aux;
		for(int i=1; i<n; i++){
			aux = curr_tour[i];
			for(int j=1; j<n; j++){
				if(i!=j){
					// Intercambio de dos nodos
					curr_tour[i] = curr_tour[j];
					curr_tour[j] = aux;

					curr_cost = simple_tour_cost(curr_tour, costs);
					// Check for the improvement
					if(curr_cost<best_cost){
						// Update if the current tour is better
						best_tour = curr_tour;
						best_cost = curr_cost;
						aux=curr_tour[i];
					}else{
						// Come back
						curr_tour = best_tour;
						curr_cost = best_cost;
					}
				}
			}
			
		}
		return best_tour;
	}


	vector<int> nn_2o_(vector<vector<float>> costs) // Nearest Neighbor + 2-opt
	{
		// ESTÁ MAL
		vector<int> curr_tour = nn(costs);
		float curr_cost = simple_tour_cost(curr_tour, costs);

		vector<int> best_tour = curr_tour;
		float best_cost = curr_cost;

		int n = costs.size(), aux;
		for(int i=1; i<n; i++){
			sh_vect_h(curr_tour,"Anterior");
			aux = curr_tour[i];
			for(int j=1; j<n; j++){
				// Intercambio de dos nodos
				curr_tour[i] = curr_tour[j];
				curr_tour[j] = aux;
			}
			sh_vect_h(curr_tour,"Actual");
			curr_cost = simple_tour_cost(curr_tour, costs);
			// Check for the improvement
			if(curr_cost<best_cost){
				// Update if the current tour is better
				best_tour = curr_tour;
				best_cost = curr_cost;
				aux=curr_tour[i];
			}else{
				// Come back
				curr_tour = best_tour;
				curr_cost = best_cost;
			}
		}

		return best_tour;
	}

	vector<int> brute_force(vector<vector<float>> costs) // Todas las combinaciones
	{
		int n = costs.size();

		// Esto es para que no se me quede atascado
		if(n>12) return vector<int>(0);

		vector<int> best_tour(n);
		for(int i=1; i<n; i++) best_tour[i]=i;
		vector<int> curr_tour = best_tour;
		float curr_cost = 0, best_cost = numeric_limits<float>::max();

		do {
			curr_cost = 0;
			for(int i=0; i<n-1; i++){
				curr_cost+=costs[curr_tour[i]][curr_tour[i+1]];
			}
			if(best_cost>curr_cost){
				best_cost = curr_cost;
				best_tour = curr_tour;
			}
		} while ( next_permutation(curr_tour.begin()+1,curr_tour.begin()+n) );

		return best_tour;
	}

	//-------------------------------------------------------------------------------------
	// TIME-WINDOW
	struct tour_var{
		vector<int> tour; // La ruta
		float cost; // El tiempo de la ruta
		int n = 0; // Número de goals alcanzados (tour.size())
	};

	tour_var time_window_tour_cost(vector<int> tour, vector<vector<float>> costs, float working_time, float tx_time, vector<float> reach_time, float critical_time)
	{
		tour_var res;
		res.tour.push_back(tour[0]);
		res.n = 1;
		res.cost = 0;

		//sh_vect_h(tour,"Entra");
		for(int i=1; i<tour.size(); i++){
			// El coste es: lo que llevo + ir al siguiente punto + trabajar + alcanzar el depot + transmitir todo lo que he recopilado
			if(critical_time>=res.cost+costs[tour[i-1]][tour[i]]+working_time+reach_time[tour[i]]+i*tx_time){
				res.cost+=(costs[tour[i-1]][tour[i]]+working_time);
				res.tour.push_back(tour[i]);
				res.n++;
			}
		}
		return res;
	}

	vector<int> nn_tw(vector<vector<float>> costs, float working_time, float tx_time, vector<float> reach_time, float critical_time) // Nearest Neighbor with Time Window
	{
		// costs: costes entre los nodos
		// reach_time: costes de  alcanzar el punto crítico del colector
		// critical_time: tiempo antes del cual hay que alcanzarlo
		int n = costs.size();
		vector<int> res;
		res.push_back(0);

		int ind = 0, ind_; // El punto de partida
		float min, cost = 0;
		vector<bool> td(n,true);
		for(int i=1; i<n; i++){
			min = numeric_limits<float>::max();
			ind_=0;
			for(int j=1; j<n; j++){
				// No hecho, coste mínimo, que permita alcanzar el punto crítico del colector y transmitir
				// (tiempo del colector < tiempo acumulado + tiempo del trabajo nuevo + alcanzar al colector + tiempo de transmitir las tareas que lleva hechas)
				if(td[j] && min>costs[ind][j] && critical_time>=cost+costs[ind][j]+working_time+reach_time[j]+i*tx_time){
					ind_ = j;
					min = costs[ind][j];
				}
			}
			if(ind_){
				res.push_back(ind_);
				cost+=(costs[ind][ind_]+working_time);
				ind = ind_;
				td[ind] = false;
			}
		}

		return res;
	}

	vector<int> nn_rs_tw(vector<vector<float>> costs, float working_time, float tx_time, vector<float> reach_time, float critical_time, float iter_time) // Nearest Neighbor + Random Shuffle with Time Window
	{
		// costs: costes entre los nodos
		// reach_time: costes de  alcanzar el punto crítico del colector
		// critical_time: tiempo antes del cual hay que alcanzarlo

		// La combinación base es la de Nearest Neighbor básico
		vector<int> base_tour = nn(costs);

		// Coste para esa combinación
		tour_var curr_var = time_window_tour_cost(base_tour, costs, working_time, tx_time, reach_time, critical_time);

		// Asi al menos no empeoro la combinación de NN
		vector<int> best_tour = nn_tw(costs, working_time, tx_time, reach_time, critical_time);
		tour_var best_var = time_window_tour_cost(best_tour, costs, working_time, tx_time, reach_time, critical_time);

		float t = 0;
		clock_t dt;
		while(t<iter_time){
			dt = clock();
			random_shuffle(base_tour.begin()+1,base_tour.end(),random_seed);

			// Obtener costes
			curr_var = time_window_tour_cost(base_tour, costs, working_time, tx_time, reach_time, critical_time);

			// Evaluar
			if(curr_var.n > best_var.n){ // Se alcanza un goal más
				best_var = curr_var;
			}else if(curr_var.n == best_var.n){ // Los goals se quedan igual
				if(curr_var.cost<best_var.cost){ // Pero es más rápido
					best_var = curr_var;
				}
			}

			dt = clock() - dt;
			t+=(float)dt/CLOCKS_PER_SEC;
		}

		return best_tour;
	}

	vector<int> nn_2o_tw(vector<vector<float>> costs, float working_time, float tx_time, vector<float> reach_time, float critical_time) // Nearest Neighbor + 2-opt with Time Window
	{
		// costs: costes entre los nodos
		// reach_time: costes de  alcanzar el punto crítico del colector
		// critical_time: tiempo antes del cual hay que alcanzarlo

		// La combinación base es la de Nearest Neighbor básico
		vector<int> base_tour = nn(costs);

		// Coste para esa combinación
		tour_var curr_var;

		// Asi al menos no empeoro la combinación de NN
		vector<int> best_tour = nn_tw(costs, working_time, tx_time, reach_time, critical_time);
		tour_var best_var = time_window_tour_cost(best_tour, costs, working_time, tx_time, reach_time, critical_time);

		// Estas 2 variables las uso porque quiero mirar las rutas enteras
		// Y best_tour y curr_var.tour almacenan rutas cortadas
		//vector<int> curr_tour = best_tour; // La ruta actual
		//vector<int> prev_tour = best_tour; // La ruta anterior

		vector<int> curr_tour = base_tour; // La ruta actual
		vector<int> prev_tour = base_tour; // La ruta anterior

		int n = costs.size(), aux;
		for(int i=1; i<n; i++){
			aux = curr_tour[i];
			for(int j=1; j<n; j++){
				// Intercambio de dos nodos
				curr_tour[i] = curr_tour[j];
				curr_tour[j] = aux;

				// Obtener costes
				curr_var = time_window_tour_cost(curr_tour, costs, working_time, tx_time, reach_time, critical_time);
				
				// Evaluar
				if(curr_var.n > best_var.n){ // Se alcanza un goal más
					best_var = curr_var;
				}else if(curr_var.n == best_var.n){ // Los goals se quedan igual
					if(curr_var.cost<best_var.cost){ // Pero es más rápido
						best_var = curr_var;
					}
				}else{ // La nueva ruta no es mejor, volver a la anterior
					curr_tour = prev_tour;
				}
			}

			
		}

		return best_tour;
	}

	vector<int> brute_force_tw(vector<vector<float>> costs, float working_time, float tx_time, vector<float> reach_time, float critical_time) // Todas las combinaciones con Time Window
	{
		int n = costs.size();

		// Esto es para que no se me quede atascado
		if(n>12) return vector<int>(0);

		vector<int> base_tour(n);
		for(int i=1; i<n; i++) base_tour[i]=i;

		tour_var curr_var = time_window_tour_cost(base_tour, costs, working_time, tx_time, reach_time, critical_time);

		vector<int> best_tour = curr_var.tour;
		float best_cost = curr_var.cost;
		int best_n = curr_var.n;

		int count = 1;

		do {
			// Costes de la permutación
			curr_var = time_window_tour_cost(base_tour, costs, working_time, tx_time, reach_time, critical_time);

			// Evaluar
			if(curr_var.n > best_n){ // Se alcanza un goal más
				best_tour = curr_var.tour;
				best_cost = curr_var.cost;
				best_n = curr_var.n;
			}else if(curr_var.n == best_n){ // Los goals se quedan igual
				if(curr_var.cost<best_cost){ // Pero es más rápido
					best_tour = curr_var.tour;
					best_cost = curr_var.cost;
					best_n = curr_var.n;
				}
			}
		} while ( next_permutation(base_tour.begin()+1, base_tour.begin()+n) );

		return best_tour;
	}

	// Función para sacar el camino
	Path path_wrapper(int tour_type, Poss<int> ini, Poss<int> goals, Poss<int> destination, float destination_time, vector<vector<float>> grid, float speed, float work_time, float tx_time, float global_path_time, vector<float> &goal_times)
	{
		// tour_type: 
		// - 0: NN, 1: NN-RS, 2: NN-2O, 3: BF     (simple)
		// - 4: NN, 5: NN-RS, 6: NN-2O, 7: BF     (time window)

		Path res;
		vector<int> tour;

		goals.x.insert(goals.x.begin(), ini.x.begin(), ini.x.end());
		goals.y.insert(goals.y.begin(), ini.y.begin(), ini.y.end());

		int ng = goals.x.size();

		vector<vector<vector<float>>> grads = compute_all_gradients(goals, grid);
		vector<vector<float>> costs = form_cost_matrix(goals, grads, speed);
		
		if(tour_type<4){
			if(tour_type==0) tour = nn(costs);
			if(tour_type==1) tour = nn_rs(costs,0.05);
			if(tour_type==2) tour = nn_2o(costs);
			if(tour_type==3) tour = brute_force(costs);
		}else{
			vector<float> reach_dest(ng,0);	
			vector<float> reach_destination = reach_critical_point(destination.x[0],destination.y[0],grads,speed);

			if(tour_type==4) tour = nn_tw(costs, work_time, tx_time, reach_destination, destination_time);
			if(tour_type==5) tour = nn_rs_tw(costs, work_time, tx_time, reach_destination, destination_time,0.05);
			if(tour_type==6) tour = nn_2o_tw(costs, work_time, tx_time, reach_destination, destination_time);
			if(tour_type==7) tour = brute_force_tw(costs, work_time, tx_time, reach_destination, destination_time);

		}

		//if(goal_times.size()!=ng) goal_times.resize(ng);

		if(tour.size())
			res = compute_path_wg(goals, tour, grads, work_time, speed, global_path_time, goal_times);

		return res;
	}

	// Lo mismo, pero utilizando un objeto para devolver las variables de visita de los goals
	struct goal_visit{
		Path path;
		vector<int> tour;
		vector<float> times;
	};

	goal_visit path_wrapper(int tour_type, Poss<int> ini, Poss<int> goals, Poss<int> destination, float destination_time, vector<vector<float>> grid, float speed, float work_time, float tx_time, float global_path_time)
	{
		// tour_type: 
		// - 0: NN, 1: NN-RS, 2: NN-2O, 3: BF     (simple)
		// - 4: NN, 5: NN-RS, 6: NN-2O, 7: BF     (time window)

		goal_visit res;

		goals.x.insert(goals.x.begin(), ini.x.begin(), ini.x.end());
		goals.y.insert(goals.y.begin(), ini.y.begin(), ini.y.end());

		int ng = goals.x.size();

		vector<vector<vector<float>>> grads = compute_all_gradients(goals, grid);
		vector<vector<float>> costs = form_cost_matrix(goals, grads, speed);
		
		if(tour_type<4){
			if(tour_type==0) res.tour = nn(costs);
			if(tour_type==1) res.tour = nn_rs(costs,0.05);
			if(tour_type==2) res.tour = nn_2o(costs);
			if(tour_type==3) res.tour = brute_force(costs);
		}else{
			vector<float> reach_dest(ng,0);	
			vector<float> reach_destination = reach_critical_point(destination.x[0],destination.y[0],grads,speed);

			if(tour_type==4) res.tour = nn_tw(costs, work_time, tx_time, reach_destination, destination_time);
			if(tour_type==5) res.tour = nn_rs_tw(costs, work_time, tx_time, reach_destination, destination_time,0.05);
			if(tour_type==6) res.tour = nn_2o_tw(costs, work_time, tx_time, reach_destination, destination_time);
			if(tour_type==7) res.tour = brute_force_tw(costs, work_time, tx_time, reach_destination, destination_time);

		}

		if(res.tour.size())
			res.path = compute_path_wg(goals, res.tour, grads, work_time, speed, global_path_time, res.times);

		return res;
	}


};

vector<vector<float>> compute_cost_matrix(Poss<int> pos, vector<vector<float>> grid, float speed)
{
	return allocation::form_cost_matrix(pos,allocation::compute_all_gradients(pos, grid), speed);
}

#endif
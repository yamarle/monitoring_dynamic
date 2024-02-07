
#ifndef GOALS_CLUSTERING_HPP
#define GOALS_CLUSTERING_HPP

// Segmentación
#include "segmentations.hpp"
#include "segmentation_functions.hpp"

#include <limits>
#include <algorithm>
#include <ctime>

namespace clustering{

	struct cluster{
		int x, y; // Posición del centroide
		Poss<int> pos; // Posiciones de los goals del cluster
	};

	Poss<int> initialize(int n, int xbase, int ybase, Poss<int> goals)
	{
		int t = goals.x.size();
		if(n>t) n=t;

		Poss<int> res; res(n+1); // La base

		res.x[0] = xbase; res.y[0] = ybase;

		int ind=0;
		for(int i=0; i<n; i++){
			// Generar aleatorio
			ind = (int(rand())/INF)*t-1;
			while(goals.x[ind]<0){
				ind = (int(rand())/INF)*t-1;
			}
			res.x[i+1] = goals.x[ind]; res.y[i+1] = goals.y[ind];
			goals.x[ind] = -1; goals.y[ind] = -1;
		}

		return res;	
	} // Inicialización de los centroides en los goals de manera aleatoria

	Poss<int> initialize(int n, int xbase, int ybase, Poss<int> goals, vector<vector<float>> grid, int iter)
	{
		int t = goals.x.size();
		if(n>t) n=t;

		Poss<int> res; res(n+1); // La base
		res.x[0] = xbase; res.y[0] = ybase;

		// Segmentar
		FMM gr(goals.x, goals.y, grid);
		grad_segm segm_obj(goals, gr.compute_gradient_());
		segm_obj.computation();
		vector<vector<int>> segments = segm_obj.get_segs();
		vector<vector<bool>> adj = compute_adjacency(segments, t);

		// Inicializar en los goals de mayor conectividad
		int ind, maxc;
		vector<int> conect;
		for(int i=0; i<n; i++){

			// Sacar la conectividad
			conect = node_connectivity(adj);

			// Seleccionar el de máxima conectividad
			maxc = 0;
			for(int j=0; j<t; j++){
				if(maxc<conect[j]){
					ind = j;
					maxc = conect[j];
				}
			}
			// Almacenar
			res.x[i+1] = goals.x[ind]; res.y[i+1] = goals.y[ind];
			// Quitar del grafo
			for(int j=0; j<t; j++){
				adj[ind][j] = 0;
				adj[j][ind] = 0;
			}
		}

		return res;	
	} // Inicilización de los centroides en los goals en base a máxima conectividad

	vector<int> goals_per_seg(Poss<int> goals, vector<vector<int>> segments, int nseg)
	{
		vector<int> res(nseg,0);	
		for(int i=0; i<goals.x.size(); i++)
			res[segments[goals.x[i]][goals.y[i]]-1]++;
		return res;
	}


	void move2far_goal(Poss<int> &pos, Poss<int> goals, vector<vector<float>> grad, vector<vector<int>> segments, int nseg)
	{
		// Encontrar el goal más alejado dentro del segmento del centroide "pos"
		Poss<int> far_pos; far_pos(nseg);
		vector<float> far(nseg,0);
		for(int i=0; i<goals.x.size(); i++){ // Todos los goals
			for(int j=0; j<pos.x.size(); j++){ // Todos menos la base
				if(segments[goals.x[i]][goals.y[i]]==segments[pos.x[j]][pos.y[j]] && far[j]<grad[goals.x[i]][goals.y[i]]){
					far[j] = grad[goals.x[i]][goals.y[i]];
					far_pos.x[j] = goals.x[i]; far_pos.y[j] = goals.y[i];
				}
			}
		}

		// Mover en la dirección de los más alejados
		Path aux;
		for(int i=0; i<nseg; i++){
			aux.gradient_descent_(grad,far_pos.x[i],far_pos.y[i]);
			if(aux.tam>1){
				pos.x[i] = aux.x[1]; pos.y[i] = aux.y[1];
			}
			aux.clear();
		}

	} // Mover hacia el goal más alejado dentro del segmento

	bool repe(Poss<int> pos, Poss<int> pos_)
	{
		for(int i=0; i<pos.x.size(); i++)
			if(pos.x[i]!=pos_.x[i] || pos.y[i]!=pos_.y[i])
				return false;
		return true;
	}

	void delete_pos(vector<vector<float>> &grid, vector<vector<float>> grad)
	{
		for(int i=0; i<grid.size(); i++)
			for(int j=0; j<grid[0].size(); j++)
				if(grad[i][j]<INF) grid[i][j] = 0;
	}

	void classify(vector<vector<int>> &segments, vector<vector<float>> grad, int nseg)
	{
		for(int i=0; i<segments.size(); i++)
			for(int j=0; j<segments[0].size(); j++)
				if(grad[i][j]<INF && !segments[i][j]) segments[i][j] = nseg;
	}

	void closest(int &x, int &y, Poss<int> pos, vector<vector<float>> grid, vector<vector<float>> base_grad)
	{
		float min=numeric_limits<float>::max();
		for(int i=0; i<pos.x.size(); i++){
			if(grid[pos.x[i]][pos.y[i]] && min>base_grad[pos.x[i]][pos.y[i]]){
				min = base_grad[pos.x[i]][pos.y[i]];
				x = pos.x[i]; y = pos.y[i];
			}
		}
	}

	int adjacent(vector<vector<float>> grad, vector<vector<float>> grid, vector<int> ngoals)
	{

	}

	vector<cluster> BGC(int n, Poss<int> goals, vector<vector<float>> grid, int xbase, int ybase, vector<vector<float>> base_grad, string folder)
	{
		//pos_coverage(int n, Poss<int> pos)
		vector<cluster> res(n+1);

		int ng = goals.x.size();
		int x, y; // Posición donde se inicializa el frente
		x = xbase; y = ybase; // Inicialmente en la base

		int ideal = ng/n; // Número idóneo de objetivos

		FMM *gr;
		vector<vector<float>> grad;

		vector<vector<float>> grid_ = grid;

		vector<vector<int>> segments(grid.size(), vector<int>(grid[0].size()));

		int nclust = 0;
		vector<int> ngoals(n+1,0);

		Poss<int> centroids;
		Poss<int> aux_pos;

		int adj;

		while(ng>=0){
			// Cubrir "ideal" objetivos
			gr = new FMM(x,y,grid);
			grad = gr->pos_coverage(ideal, goals, aux_pos);
			delete(gr);
			// Eliminar el espacio cubierto del grid
			delete_pos(grid,grad);

			if(aux_pos.x.size()>ideal/2){ // Se considera que es un cluster nuevo
				// Clasificar los goals del cluster
				res[nclust].x = x; res[nclust].y = y;
				ngoals[nclust] += aux_pos.x.size();
				ng-=aux_pos.x.size();

				//aux_pos.show_();

				res[nclust].pos.append(aux_pos);

				/*
				//---------------------------------------------------------------------------------
				// Guardar las iteraciones
				centroids.x.push_back(x); centroids.y.push_back(y);
				centroids.save((folder+"centroids_"+to_string(nclust)+".txt").c_str());
				//goals.save((folder+"goals_"+to_string(i)+".txt").c_str());
				res[nclust].pos.save((folder+"goals_"+to_string(nclust)+".txt").c_str());
				//save_vect((folder+"ng_"+to_string(nclust)+".txt").c_str(),ngoals);
				//---------------------------------------------------------------------------------
				*/

				if(nclust<n) nclust++; // Siguiente cluster
				aux_pos.clear();
				
			}else{ // Se pega a un cluster adyacente
				// El índice del adyacente
				adj = adjacent(grad, grid_, ngoals);
				ng--;

			}

			// Encontrar el siguiente más cercano a la base
			closest(x,y,goals,grid,base_grad);

		}

		// Resultados

		

		return res;

	}

	vector<cluster> k_means(int n, Poss<int> goals, vector<vector<float>> grid, int xbase, int ybase, int iter, string folder)
	{
		vector<cluster> res(n+1);

		// 1. Inicializar los centroides
		//Poss<int> centroids = initialize(n, xbase, ybase, goals); // Inicialización aleatoria
		Poss<int> centroids = initialize(n, xbase, ybase, goals, grid, iter); // Inicialización en los de mayor conectividad
		vector<Poss<int>> buff(2,centroids); // Variable que almacena las posiciones anteriores

		vector<vector<float>> grad;
		vector<vector<int>> segments;

		vector<int> ngoals(n+1,0);

		int nseg;
		FMM *gr;
		grad_segm *gs;
		for(int i=0; i<iter; i++){
			//cout<<"Iteración: "<<i<<endl;
			// 2. Clasificar los goals en clusters
			// Obtener el gradiente
			gr = new FMM(centroids.x, centroids.y, grid);
			grad = gr->compute_gradient_();
			delete(gr);
			// Obtener los segmentos
			gs = new grad_segm(centroids, grad);
			gs->computation();
			segments = gs->get_segs();
			delete(gs);
			// 3. Mover en dirección al goal más alejado del cluster
			move2far_goal(centroids, goals, grad, segments, n+1);

			ngoals = goals_per_seg(goals, segments, n+1);

			// Comprobar repetición de posiciones
			if(repe(centroids,buff[0])){
				break;
			}else{
				buff[0] = buff[1]; buff[1] = centroids;
			}

			/*
			//---------------------------------------------------------------------------------
			// Guardar las iteraciones
			centroids.save((folder+"centroids_"+to_string(i)+".txt").c_str());
			//goals.save((folder+"goals_"+to_string(i)+".txt").c_str());
			for(int j=0; j<n+1; j++){
				res[j].pos = points_in_seg(segments, goals, j+1); // goals de cada segmento
				res[j].pos.save((folder+"goals_"+to_string(i)+"_"+to_string(j)+".txt").c_str());
			}
			save_vect((folder+"ng_"+to_string(i)+".txt").c_str(),ngoals);
			//---------------------------------------------------------------------------------
			*/

		}

		// Clasificar
		for(int i=0; i<n+1; i++){
			// Centroide
			res[i].x = centroids.x[i]; res[i].y = centroids.y[i];
			// Goals
			res[i].pos = points_in_seg(segments, goals, i+1);
		}

		return res;	
	} // K-means normal con obstáculos

	vector<cluster> k_means_sem(int n, Poss<int> goals, vector<vector<float>> grid, int xbase, int ybase, int iter, string folder)
	{
		vector<cluster> res(n+1);

		// 1. Inicializar los entroides
		//Poss<int> centroids = initialize(n, xbase, ybase, goals); // Inicialización aleatoria
		Poss<int> centroids = initialize(n, xbase, ybase, goals, grid, iter); // Inicialización en los de mayor conectividad
		vector<Poss<int>> buff(2,centroids); // Variable que almacena las posiciones anteriores

		vector<vector<float>> grad;
		vector<vector<int>> segments;

		vector<int> ngoals(n+1,0); // Cantidad de goals que hay dentro de cada cluster
		int ideal = goals.x.size()/(n+1); // Número ideal de goals

		int max_dev_clust; // Índice del cluster que se desvía más del ideal
		int max_dev; // desviación máxima

		vector<bool> to_move(n+1,true);
		to_move[0] = false; // El centroide de la base no se mueve nunca

		int nseg; // cantidad de segmentos/clusters
		FMM *gr;
		grad_segm *gs;
		for(int i=0; i<iter; i++){
			//cout<<"Iteración: "<<i<<endl;

			// 2. Clasificar los goals en clusters
			// Obtener el gradiente
			gr = new FMM(centroids.x, centroids.y, grid);
			grad = gr->compute_gradient_();
			delete(gr);

			// Obtener los segmentos
			gs = new grad_segm(centroids, grad);
			gs->computation();
			segments = gs->get_segs();
			delete(gs);
			// 3. Mover en dirección al goal más alejado del cluster
			move2far_goal(centroids, goals, grad, segments, n+1);

			ngoals = goals_per_seg(goals, segments, n+1);

			max_dev = 0;
			for(int j=1; j<n+1; j++){ // No miro la base
				if(max_dev<ngoals[j]-ideal){
					max_dev=ngoals[j]-ideal;
					max_dev_clust = j;
				}
			}

			// Comprobar repetición de posiciones
			if(repe(centroids,buff[0])){
				break;
			}else{
				buff[0] = buff[1]; buff[1] = centroids;
			}

			/*
			//---------------------------------------------------------------------------------
			// Guardar las iteraciones
			centroids.save((folder+"centroids_"+to_string(i)+".txt").c_str());
			//goals.save((folder+"goals_"+to_string(i)+".txt").c_str());
			for(int j=0; j<n+1; j++){
				res[j].pos = points_in_seg(segments, goals, j+1); // goals de cada segmento
				res[j].pos.save((folder+"goals_"+to_string(i)+"_"+to_string(j)+".txt").c_str());
			}
			//save_matr((folder+"grad_"+to_string(i)+".txt").c_str(),grad);
			save_vect((folder+"ng_"+to_string(i)+".txt").c_str(),ngoals);
			//---------------------------------------------------------------------------------
			*/
		}

		// Clasificar
		for(int i=0; i<n+1; i++){
			// Centroide
			res[i].x = centroids.x[i]; res[i].y = centroids.y[i];
			// Goals
			res[i].pos = points_in_seg(segments, goals, i+1);
		}

		return res;	
	} // K-means con obstáculos y paradas de algunos centroides

	void save(string folder_name, vector<cluster> clusters)
	{
		int n = clusters.size();

		// Número de goals
		vector<int> goals(n,0);
		for(int i=0; i<n; i++)
			goals[i]+=clusters[i].pos.x.size();
		save_vect((folder_name+"number.txt").c_str(),goals);

		// Centroides
		Poss<int> centr; centr(n);
		for(int i=0; i<n; i++){
			centr.x[i] = clusters[i].x;
			centr.y[i] = clusters[i].y;
		}
		centr.save((folder_name+"centroids.txt").c_str());

		// Posiciones
		for(int i=0; i<n; i++){
			clusters[i].pos.save((folder_name+"pos_"+to_string(i)+".txt").c_str());
		}

	}

}

#endif

#ifndef COLLECTOR_FUNCTIONS_HPP
#define COLLECTOR_FUNCTIONS_HPP

// Para partir el entorno del colector en base a las áreas de trabajo(segmentos) de los trabajadores
#include "segmentations.hpp"
#include "segmentation_functions.hpp"

// Para estimar el recorrido de los trabajadores dentro de sus segmentos
#include "allocations.hpp"

// Guardar sobretodo
#include "useless.hpp"

namespace collector_functions{

	//------------------------------------------------------------------------------------------
	// Funciones de búsqueda de los destinos de los colectores
	void clear_distance(vector<vector<float>> &grad, int x, int y)
	{
		int val = (int)grad[x][y];
		for(int i=x-val; i<x+val; i++){
			for(int j=y-val; j<y+val; j++){
				grad[i][j]=INF;
			}
		}
	} // Anular los puntos (fijar como obstáculos) con el valor del gradiente

	// Inicializar centroides
	Poss<int> maximum_initialization(int n, vector<vector<float>> obst_grad)
	{
		Poss<int> res;;
		int x, y;
		
		for(int i=0; i<n; i++){
			// Encontrar el máximo
			x = -1; y = -1;
			find_matr_max(obst_grad,x,y);
			if(x<0) break;
			res.x.push_back(x); res.y.push_back(y);
			clear_distance(obst_grad,x,y);
		}

		return res;
	} // Inicialización de los centroides de los colectores sobre los máximos del gradiente (más lejanos a los obstaçulos)

	void random_initialization(int n, vector<vector<float>> grid, Poss<int> &pos)
	{
		Poss<int> aux = gen_pos2(grid, n);
		pos.append(aux);
	} // Inicialización aleatoria

	Poss<int> maximum_connectivity_initialization(int n, vector<int> connectivity, Poss<int> centr, vector<vector<float>> distance_gradient)
	{
		Poss<int> res;

		int max_con;
		float min_dist;
		int ind;
		for(int i=0; i<n; i++){
			// Encontrar el de máxima conectividad
			max_con = -1;
			for(int j=0; j<connectivity.size(); j++){
				if(max_con<connectivity[j]){
					max_con = connectivity[j];
					ind = j;
				}
			}
			if(max_con>=0){
				// Comprobar si hay varios de la misma conectividad seleccionar el de menor distancia (más cercano a la base)
				min_dist = numeric_limits<float>::max();
				for(int j=0; j<connectivity.size(); j++){
					if(max_con == connectivity[j] && min_dist>distance_gradient[centr.x[j]][centr.y[j]]){
						min_dist = distance_gradient[centr.x[j]][centr.y[j]];
						ind = j;
					}
				}

				res.x.push_back(centr.x[ind]);
				res.y.push_back(centr.y[ind]);
				connectivity[ind] = -1;
			}else{
				return res;
			}
		}

		return res;
	} // Inicialización de los centroides sobre los centroides de los trabajadores de máxima conectividad

	Poss<int> maximum_connectivity_initialization(int n, vector<int> connectivity, Poss<int> centr, vector<vector<float>> distance_gradient, vector<bool> discarded)
	{
		Poss<int> res;

		int max_con;
		float min_dist;
		int ind;
		for(int i=0; i<n; i++){
			// Encontrar el de máxima conectividad
			max_con = -1;
			for(int j=0; j<connectivity.size(); j++){
				if(max_con<connectivity[j] && !discarded[j]){
					max_con = connectivity[j];
					ind = j;
				}
			}
			if(max_con>=0){
				// Comprobar si hay varios de la misma conectividad seleccionar el de menor distancia (más cercano a la base)
				min_dist = numeric_limits<float>::max();
				for(int j=0; j<connectivity.size(); j++){
					if(max_con == connectivity[j] && !discarded[j] && min_dist>distance_gradient[centr.x[j]][centr.y[j]]){
						min_dist = distance_gradient[centr.x[j]][centr.y[j]];
						ind = j;
					}
				}

				res.x.push_back(centr.x[ind]);
				res.y.push_back(centr.y[ind]);
				connectivity[ind] = -1;
			}else{
				return res;
			}
		}

		return res;
	} // Inicialización de los centroides sobre los centroides de los trabajadores de máxima conectividad

	Poss<int> maximum_area_initialization(int n, vector<int> areas, Poss<int> centr, vector<bool> discard)
	{
		Poss<int> res;

		int max_area = 0;
		int ind;
		for(int i=0; i<n; i++){
			max_area = 0;
			for(int j=0; j<areas.size(); j++){
				if(!discard[j] && max_area<areas[j]){
					max_area = areas[j];
					ind = j;
				}
			}
			if(max_area){
				discard[ind] = true;
				res.x.push_back(centr.x[ind]);
				res.y.push_back(centr.y[ind]);
			}else{
				break;
			}
		}

		return res;
	}

	void move2far(Poss<int> &pos, Poss<int> dest, vector<vector<float>> grad)
	{
		Path camino;
		for(int i=1; i<pos.x.size(); i++){ // LA PRIMERA POSICION ES LA BASE
			camino.gradient_descent_(grad,dest.x[i],dest.y[i]);
			if(camino.tam>1){
				pos.x[i]=camino.x[1]; pos.y[i]=camino.y[1];
			}
			camino.clear();
		}
	} // Mover en dirección al punto más alejado del segmento

	void move2far(Poss<int> &pos, Poss<int> &coll, Poss<int> dest, vector<vector<float>> grad)
	{
		Path camino;
		int ind;
		for(int i=1; i<pos.x.size(); i++){ // LA PRIMERA POSICION ES LA BASE
			camino.gradient_descent_(grad,dest.x[i],dest.y[i]);
			if(camino.tam>1){
				pos.x[i]=camino.x[1]; pos.y[i]=camino.y[1];
				// Punto que está justo en el medio
				camino.compute_distance();
				ind = camino.find_d(camino.d/2);
				coll.x[i]=camino.x[ind]; coll.y[i]=camino.y[ind];
			}
			camino.clear();
		}
	} // Mover en dirección al punto más alejado del segmento

	vector<float> compute_distance2depot(Poss<int> depot, Poss<int> pos, vector<vector<float>> grid, vector<int> alloc)
	{
		vector<float> res(pos.x.size(),0);
		for(int i=0; i<depot.x.size(); i++){
			FMM gr(depot.x[i],depot.y[i],grid);
			vector<vector<float>> grad = gr.compute_gradient_();
			for(int j=0; j<alloc.size(); j++){
				if(alloc[j]==i){
					res[j] = grad[pos.x[j]][pos.y[j]];
				}
			}
		}
		return res;
	} // Obtener las distancias hasta los puntos de enctuntro

	//------------------------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------
	// Funciones de clasificación
	vector<int> pos_allocation(Poss<int> pos, vector<vector<int>> segm)
	{
		vector<int> res(pos.x.size());
		for(int i=0; i<pos.x.size(); i++){
			if(segm[pos.x[i]][pos.y[i]]) res[segm[pos.x[i]][pos.y[i]]-1];
		}
		return res;
	}

	vector<vector<float>> coincidence_per(vector<vector<int>> cseg, vector<vector<int>> wseg, int cs, int ws)
	{
		vector<vector<float>> res(cs,vector<float>(ws,0));
		vector<int> wa(ws,0); // área de segmento de trabajadores
		//vector<int> ca(ws,0); // área de segmento de colectores
		for(int i=0; i<cseg.size(); i++){
			for(int j=0; j<cseg[0].size(); j++){
				if(cseg[i][j]>0 && wseg[i][j]>0){ // No es un obstáculo
					wa[wseg[i][j]-1]++; // Area del segmento
					res[cseg[i][j]-2][wseg[i][j]-1]++; // Área que está contenida dentro de los segmentos de los colectores
				}
			}
		}
		for(int i=0; i<cs; i++)
			for(int j=0; j<ws; j++)
				res[i][j]/=float(wa[j]);
		return res;
	} // Porcentaje de coincidencia de los segmentos de los trabajadores con los de los colectores

	vector<vector<int>> coincidence(vector<vector<int>> cseg, vector<vector<int>> wseg, int cs, int ws)
	{
		vector<vector<int>> res(cs,vector<int>(ws,0));
		//vector<int> ca(ws,0); // área de segmento de colectores
		for(int i=0; i<cseg.size(); i++){
			for(int j=0; j<cseg[0].size(); j++){
				if(cseg[i][j]>0 && wseg[i][j]>0){ // No es un obstáculo
					res[cseg[i][j]-2][wseg[i][j]-1]++; // Área que está contenida dentro de los segmentos de los colectores
				}
			}
		}
		return res;
	} // Porcentaje de coincidencia de los segmentos de los trabajadores con los de los colectores

	vector<int> allocation(vector<vector<int>> coincidence)
	{
		int nc = coincidence.size(), nw = coincidence[0].size();
		vector<int> res(coincidence[0].size());
		// Clasificar los trabajadores en base a la máxima coincidencia
		int max;
		for(int i=0; i<nw; i++){
			max = 0;
			for(int j=0; j<nc; j++){
				if(max<coincidence[j][i]){
					max = coincidence[j][i];
					res[i] = j+1; // Esto es para que no se confunda con los que van directamente a la base
				}
			}
		}

		return res;
	} // Asignar al segmento que tiene máxima coincidencia

	//------------------------------------------------------------------------------------------

	vector<float> cost2col_vector(vector<vector<vector<float>>> grads, Poss<int> col_pos, vector<int> alloc, int col, float speed)
	{
		// Recibe:
		// - grads: gradientes de los trabajadores
		// - col_pos: posicipnes de destino de los colectores
		// - alloc: asignaciones
		// - speed: velocidad de los rtabajadores
		vector<float> res;

		for(int i=0; i<alloc.size(); i++){
			if(alloc[i]==col){
				res.push_back(grads[i][col_pos.x[col]][col_pos.y[col]]/speed); // La primera es la base
			}
		}

		return res;	
	} // Formar las matrices de coste de llegada al punto de destino del colector

	vector<int> goal_selection(vector<int> goals, vector<int> alloc, int col)
	{
		vector<int> res;
		for(int i=0; i<goals.size(); i++){
			if(alloc[i]==col){
				res.push_back(goals[i]);
			}
		}
		return res;
	} // Seleccionar los goals de los trabajadores para el colecotr "col"

	int find_closest2base(Poss<int> pos, vector<vector<float>> base_grad, vector<int> alloc)
	{
		int res;
		float min = numeric_limits<float>::max();
		for(int i=0; i<alloc.size(); i++){
			if(alloc[i] && min>base_grad[pos.x[i]][pos.y[i]]){
				min=base_grad[pos.x[i]][pos.y[i]];
				res = i;
			}
		}

		return res;
	} // Encontrar el más cercano a la base, distancia geodésica

	int find_closest2base(Poss<int> pos, vector<vector<float>> base_grad, vector<int> alloc, vector<int> depths, int depth)
	{
		int res;
		float min = numeric_limits<float>::max();
		for(int i=0; i<alloc.size(); i++){
			if(depths[i]==depth && min>base_grad[pos.x[i]][pos.y[i]]){
				min=base_grad[pos.x[i]][pos.y[i]];
				res = i;
			}
		}

		return res;
	} // Encontrar el más cercano a la base de la profundidad "depth", distancia geodésica

	vector<int> allocated_areas(int ncolseg, vector<int> alloc, vector<int> worker_areas)
	{
		vector<int> res(ncolseg,0);	

		for(int i=0; i<alloc.size(); i++){
			res[alloc[i]]+=worker_areas[i];
		}

		return res;
	}

	float maximum_area_deviation(vector<int> areas, int total, float ideal)
	{
		float res = 0, val;
		for(int i=0; i<areas.size(); i++){
			val = ideal-(float)areas[i]/(float)total;
			if(res<val) res = val;
		}
		return res;
	}

	float maximum_deviation_between_areas(vector<int> areas, vector<int> alloc, int n, float ideal)
	{
		float res = 0;
		int val = 0;
		for(int i=0; i<areas.size(); i++){
			if(alloc[i])
			for(int j=0; j<areas.size(); j++){
				if(alloc[j] && val<areas[i]-areas[j]){
					val = areas[i]-areas[j];
				}
			}
		}
		return res;
	}

	vector<vector<int>> collector_segmentation(Poss<int> &positions, vector<int> &areas, vector<vector<float>> grid, int iter_max)
	{
		vector<vector<int>> segments;

		FMM *gr;
		grad_segm *gs;

		Poss<int> coll_segm_far;

		vector<vector<float>> coll_grad;

		int iter = 0;
		while(iter<iter_max){
			// 1. Obtener gradiente
			gr = new FMM(positions.x, positions.y, grid);
			coll_grad = gr->compute_gradient_();
			delete(gr);
			// 2. Obtener los segmentos
			gs = new grad_segm(positions, coll_grad);
			gs->computation();
			segments = gs->get_segs();
			coll_segm_far = gs->get_far_pos(); areas = gs->get_areas();
			delete(gs);
			// 3. Mover
			move2far(positions, coll_segm_far, coll_grad);

			/*
			// Guardar las iteraciones de los movimientos de los centroides de los colectores
			save_matr((folder_name+"coll_segs"+to_string(iter)+".txt").c_str(),coll_segments);
			save_matr((folder_name+"coll_grad"+to_string(iter)+".txt").c_str(),coll_grad);
			positions.save((folder_name+"coll_far"+to_string(iter)+".txt").c_str());
			coll_pos.save((folder_name+"coll_pos"+to_string(iter)+".txt").c_str());
			*/

			iter++;
		}


		return segments;
	}

	vector<vector<int>> collector_segmentation(Poss<int> &positions, vector<int> &areas, vector<vector<float>> &coll_grad, vector<vector<float>> grid, int iter_max)
	{
		vector<vector<int>> segments;

		FMM *gr;
		grad_segm *gs;

		Poss<int> coll_segm_far;

		int iter = 0;
		while(iter<iter_max){
			// 1. Obtener gradiente
			gr = new FMM(positions.x, positions.y, grid);
			coll_grad = gr->compute_gradient_();
			delete(gr);
			// 2. Obtener los segmentos
			gs = new grad_segm(positions, coll_grad);
			gs->computation();
			segments = gs->get_segs();
			coll_segm_far = gs->get_far_pos(); areas = gs->get_areas();
			delete(gs);
			// 3. Mover
			move2far(positions, coll_segm_far, coll_grad);

			/*
			// Guardar las iteraciones de los movimientos de los centroides de los colectores
			save_matr((folder_name+"coll_segs"+to_string(iter)+".txt").c_str(),coll_segments);
			save_matr((folder_name+"coll_grad"+to_string(iter)+".txt").c_str(),coll_grad);
			positions.save((folder_name+"coll_far"+to_string(iter)+".txt").c_str());
			coll_pos.save((folder_name+"coll_pos"+to_string(iter)+".txt").c_str());
			*/

			iter++;
		}


		return segments;
	}

	void select_critical_point(int &x, int &y, float &t, float time_limit, Path collector, Poss<int> segment_contour, vector<vector<float>> grid)
	{

		// RECUERDA QUE DEBE RECIBIR EL CAMINO DEL COLECTOR CUANDO VA A LA BASE

		// - time_limit es el tiempo que se necesita para transmitir los datos

		FMM gr(segment_contour.x,segment_contour.y,grid);
		vector<vector<float>> grad = gr.compute_gradient_();

		// Encontrar el punto más cercano desde el segmento al colector
		float min = numeric_limits<float>::max(), tval;
		int ind;
		for(int i=0; i<collector.tam; i++){
			if(min>grad[collector.x[i]][collector.y[i]]){
				min = grad[collector.x[i]][collector.y[i]];
				ind = i;
				tval = collector.tc[i]; // El tiempo del punto más cercano
			}
		}

		// He encontrado el más cercano
		// Ahora quiero ir hasta el tiempo que he fijado
		for(int i=ind; i<collector.tam; i++){
			if(collector.tc[ind]-tval<time_limit){
				x = collector.x[i]; y = collector.y[i];
				t = collector.tc[i];
			}
		}

	} // Seleccionar el punto crítico

	vector<Path> cycle_formation(Path ida, float tw)
	{
		vector<Path> res(3);
		res[0]=ida;
		res[1]=ida.back();
		res[2].x.push_back(ida.x[0]); res[2].y.push_back(ida.y[0]);
		res[2].x.push_back(ida.x[0]); res[2].y.push_back(ida.y[0]);
		res[2].dpp.push_back(0); res[2].dpp.push_back(0);
		res[2].tpp.push_back(0); res[2].tpp.push_back(tw);
		res[2].dc.push_back(0); res[2].dc.push_back(0);
		res[2].tc.push_back(res[1].t); res[2].tc.push_back(res[1].t+tw);
		res[2].t=res[1].t+tw;
		res[2].tam=2;
		return res;
	} // Formación de un ciclo del colector (ida, vuelta y espera)

	vector<Path> increase_cycle_time(vector<Path> cycle, float t)
	{
		for(int i=0; i<cycle.size(); i++){
			cycle[i].initial_time(t);
		}
		return cycle;
	} // "Subir el ciclo en tiempo"

	float periodo_del_colector(vector<Path> colector, float time)
	{
		float res=0;

		float duracion = colector[2].t-colector[0].tc[0]; // Del ciclo completo
		float margen = colector[2].t - colector[1].t; // El tiempo del tramo de transmisión
		float ini = colector[0].tc[0];
		float fin = colector[2].t;

		while(fin<time){
			ini = res*duracion;
			fin = (res+1)*duracion;
			res++;
			if(time<=fin && time>=ini){
				if(time>=fin-margen){
					res++;
					ini = res*duracion;
					fin = (res+1)*duracion;
				}
				break;
			}
		}

		return res;
	} // El periodo del colector con respecto un tiempo "time"

	void append_cycle(vector<Path> &cycles)
	{
		if(cycles.size()>2){
			double tini = cycles[cycles.size()-1].t;
			for(int i=0; i<3; i++){
				cycles.push_back(cycles[i]);
				cycles[cycles.size()-1].initial_time(tini);
			}
		}
	} // añadir un cilo al final

}




#endif
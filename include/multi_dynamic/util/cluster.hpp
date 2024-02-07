#ifndef CLUSTER_HPP
#define CLUSTER_HPP

#include "useless.hpp"
#include "geometry.hpp"

namespace mean_shift{
	vector<vector<float>> simple_movement(vector<vector<float>> points, vector<vector<float>> grid, float radio); // Movimiento simple del mean-shift
	vector<vector<vector<float>>> simple_movement_it(vector<vector<float>> points, vector<vector<float>> grid, float radio); // Movimiento simple del mean-shift (devuelve todas las iteraciones)
	vector<vector<float>> weighted_movement(vector<vector<float>> points, vector<vector<float>> grid, vector<vector<float>> grad, float radio); // Moviemiento ponderado del mean-shift
	vector<vector<vector<float>>> weighted_movement_it(vector<vector<float>> points, vector<vector<float>> grid, vector<vector<float>> grad, float radio); // Moviemiento ponderado del mean-shift (devuelve todas las iteraciones)
}

namespace dist_shift{
	vector<vector<float>> movement(vector<vector<float>> grid, vector<vector<float>> dist, float radio);
	vector<vector<vector<float>>> movement_it(vector<vector<float>> grid, vector<vector<float>> dist, float radio);
}

namespace adjacency_clustering{
	template <typename T>
	vector<Poss<T>> cluster(Poss<T> points, vector<vector<float>> grid)
	{
	    vector<Poss<T>> res;
	    if(points.x.size() == 0) return res;

	    res.resize(1);

	    res[0].x.push_back(points.x[0]);
	    res[0].y.push_back(points.y[0]);
	    points.erase(0);

	    for(int i=0; i<points.x.size(); i++) grid[points.x[i]][points.y[i]] = 2;

	    Poss<T> q = res[0];
	    int x, y;
	    while(q.x.size()){
	        for(int i=0; i<geometry::nx.size(); i++){
	            x = q.x[0] + geometry::nx[i]; y = q.y[0] + geometry::ny[i];
	            if(grid[x][y] == 2){ // Es un vecino
	                q.x.push_back(x); q.y.push_back(y); // Insertar
	                // Insertar en el 
	                res[res.size()-1].x.push_back(x);
	                res[res.size()-1].y.push_back(y);
	                grid[x][y] = 1; // NO volver a elegir
	                // Eliminar el punto de la lista de puntos
	                for(int j=0; j<points.x.size(); j++)
	                    if(points.x[j] == x && points.y[j] == y){
	                        points.erase(j);
	                        break;
	                    }
	            }
	        }
	        q.erase(0); // Eliminar el primero
	        if(!q.x.size()){ // NO quedan puntos en cola
	            if(points.x.size()){ // Pero quedan puntos por clasificar
	                res.resize(res.size()+1); // Nuevo cluster
	                res[res.size()-1].x.push_back(points.x[0]);
	                res[res.size()-1].y.push_back(points.y[0]);
	                grid[points.x[0]][points.y[0]] = 1;
	                q.x.push_back(points.x[0]); q.y.push_back(points.y[0]);
	                points.erase(0);
	            }else{ // NO quedan puntos por clasificar
	                return res; // Todos los puntos clasificados
	            }
	        }
	    }
	    return res;
	}
}

#endif
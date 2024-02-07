#ifndef GOAL_FUNCTIONS_HPP
#define GOAL_FUNCTIONS_HPP

#include "fmm_2.hpp"  // Uso functions.hpp para cargar/guardar
#include <chrono>  // Semillas para la generación aleatoria
#include <random>  // Distribuciones

namespace goals{

	
	// ---------------------------------------------------------------------------------------------------------------
	//       GENERACIÓN CON DEISTRIBUCIÓN UNIFORME
	// ---------------------------------------------------------------------------------------------------------------

	Poss<int> uniform_generation(vector<vector<float>> free_space, int n, int seed)
	{
		// Recibe:
		// - free_space: gradiente del espacio libre (el de la base calculado al principio de todo)
		// - n: número de goals a generar en todo el escenario

		Poss<int> res; res(n);
		int sx=free_space.size(), sy=free_space[0].size();
		// Generador de aleatorios
		//unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine generator(seed);
		std::uniform_int_distribution<int> distribution_x(0,sx-1), distribution_y(0,sy-1);
		for(int i=0; i<n; i++){
			res.x[i] = distribution_x(generator);
			res.y[i] = distribution_y(generator);
			while(free_space[res.x[i]][res.y[i]]==INF){
				res.x[i] = distribution_x(generator);
				res.y[i] = distribution_y(generator);
			}
			free_space[res.x[i]][res.y[i]]=INF;
		}
		return res;
	} // Generación uniforme de los goals, en todo el escenario (Le paso la semilla)

	Poss<int> uniform_generation(vector<vector<float>> free_space, int n)
	{
		// Recibe:
		// - free_space: gradiente del espacio libre (el de la base calculado al principio de todo)
		// - n: número de goals a generar en todo el escenario

		Poss<int> res; res(n);
		int sx=free_space.size(), sy=free_space[0].size();
		// Generador de aleatorios
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine generator(seed);
		std::uniform_int_distribution<int> distribution_x(0,sx-1), distribution_y(0,sy-1);
		for(int i=0; i<n; i++){
			res.x[i] = distribution_x(generator);
			res.y[i] = distribution_y(generator);
			while(free_space[res.x[i]][res.y[i]]==INF){
				res.x[i] = distribution_x(generator);
				res.y[i] = distribution_y(generator);
			}
			free_space[res.x[i]][res.y[i]]=INF;
		}
		return res;
	} // Generación uniforme de los goals, en todo el escenario (Semilla aleatoria)

	Poss<int> total_uniform_goals(vector<vector<float>> free_space, int n, int nit)
	{
		Poss<int> res;
		for(int i=0; i<nit; i++){
			res.append(uniform_generation(free_space, n));
		}
		return res;
	} // Generar goals con distribución uniforme por tandas

	Poss<int> total_uniform_goals(vector<vector<float>> free_space, int n, int nit, int seed)
	{
		Poss<int> res;
		for(int i=0; i<nit; i++){
			res.append(uniform_generation(free_space, n, seed));
		}
		return res;
	} // Generar goals con distribución uniforme por tandas

	Poss<int> total_uniform_goals(vector<vector<float>> free_space, int n, int nit, vector<int> seeds)
	{
		Poss<int> res;
		for(int i=0; i<nit; i++){
			res.append(uniform_generation(free_space, n, seeds[i]));
		}
		return res;
	} // Generar goals con distribución uniforme por tandas

	// ---------------------------------------------------------------------------------------------------------------
	//       GENERACIÓN CON DEISTRIBUCIÓN GAUSSIANA
	// ---------------------------------------------------------------------------------------------------------------

	Poss<int> gaussian_generation(vector<vector<float>> free_space, Poss<int> centr, vector<float> deviation, int n, int nseg, int seed)
	{
		// Recibe:
		// - free_space: gradiente del espacio libre (el de la base calculado al principio de todo)
		// - centr: centroides de donde se quieren generar los goals
		// - deviation: la desviación en distancia desde los centroides (probablemente será el radio de segmento, radio euclídeo sin obstáculos)
		// - n: número de goals a generar en todo el escenario
		// - nseg: número de segmentos

		int sx=free_space.size(), sy=free_space[0].size();
		//unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	    std::default_random_engine generator(seed);
	    Poss<int> res;
	    int x_,y_;
	    float dist;
	    int cont;
	    // Ahora genero por segmentos
	    for(int i=0; i<nseg; i++){
	    	if(deviation[i]>1){
	    		std::normal_distribution<float> distr_x((float)centr.x[i],deviation[i]), distr_y((float)centr.y[i],deviation[i]);
		    	for(int j=0; j<n/nseg; j++){
		    		cont=0;
		    		x_=(int)distr_x(generator); y_=(int)distr_y(generator);
		    		dist=sqrt(pow(x_-centr.x[i],2)+pow(y_-centr.y[i],2));
		    		x_=x_>-1?x_:0; x_=x_<sx?x_:sx-1;
		    		y_=y_>-1?y_:0; y_=y_<sy?y_:sy-1;
		    		while(free_space[x_][y_]==INF || dist>deviation[i]){
		    			x_=(int)distr_x(generator); y_=(int)distr_y(generator);
		    			x_=x_>-1?x_:0; x_=x_<sx?x_:sx-1;
		    			y_=y_>-1?y_:0; y_=y_<sy?y_:sy-1;
		    			dist=sqrt(pow(x_-centr.x[i],2)+pow(y_-centr.y[i],2));
		    			cont++;
		    			if(cont==1000) return res;
		    		}
		    		res.x.push_back(x_); res.y.push_back(y_);
		    		free_space[x_][y_]=INF;
		    	}
		    }
	    }
	    return res;
	} // Generación aleatoria de posiciones utilizando la distribución gaussiana (Le paso la semilla)

	Poss<int> gaussian_generation(vector<vector<float>> free_space, Poss<int> centr, vector<float> deviation, int n, int nseg)
	{
		// Recibe:
		// - free_space: gradiente del espacio libre (el de la base calculado al principio de todo)
		// - centr: centroides de donde se quieren generar los goals
		// - deviation: la desviación en distancia desde los centroides (probablemente será el radio de segmento, radio euclídeo sin obstáculos)
		// - n: número de goals a generar en todo el escenario
		// - nseg: número de segmentos

		int sx=free_space.size(), sy=free_space[0].size();
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	    std::default_random_engine generator(seed);
	    Poss<int> res;
	    int x_,y_;
	    float dist;
	    int cont;
	    // Ahora genero por segmentos
	    for(int i=0; i<nseg; i++){
	    	if(deviation[i]>1){
	    		std::normal_distribution<float> distr_x((float)centr.x[i],deviation[i]), distr_y((float)centr.y[i],deviation[i]);
		    	for(int j=0; j<n/nseg; j++){
		    		cont=0;
		    		x_=(int)distr_x(generator); y_=(int)distr_y(generator);
		    		dist=sqrt(pow(x_-centr.x[i],2)+pow(y_-centr.y[i],2));
		    		x_=x_>-1?x_:0; x_=x_<sx?x_:sx-1;
		    		y_=y_>-1?y_:0; y_=y_<sy?y_:sy-1;
		    		while(free_space[x_][y_]==INF || dist>deviation[i]){
		    			x_=(int)distr_x(generator); y_=(int)distr_y(generator);
		    			x_=x_>-1?x_:0; x_=x_<sx?x_:sx-1;
		    			y_=y_>-1?y_:0; y_=y_<sy?y_:sy-1;
		    			dist=sqrt(pow(x_-centr.x[i],2)+pow(y_-centr.y[i],2));
		    			cont++;
		    			if(cont==1000) return res;
		    		}
		    		res.x.push_back(x_); res.y.push_back(y_);
		    		free_space[x_][y_]=INF;
		    	}
		    }
	    }
	    return res;
	} // Generación aleatoria de posiciones utilizando la distribución gaussiana (Semilla aleatoria)

	Poss<int> total_gaussian_goals(vector<vector<float>> free_space, int n, int nit, int nseg, Poss<int> centr, vector<float> deviation)
	{
		Poss<int> res;
		for(int i=0; i<nit; i++){
			res.append(gaussian_generation(free_space,centr,deviation,n,nseg));
		}
		return res;
	} // Generar goals con distribución gaussiana por tandas

	// ---------------------------------------------------------------------------------------------------------------
	//       FICHEROS
	// ---------------------------------------------------------------------------------------------------------------

	void save(const char* filename, Poss<int> goals, vector<float> t_gen, vector<float> t_rx){
		ofstream file(filename); // abrir para escribir
		string line;
		if(file.is_open()){
			for(int i=0; i<goals.x.size(); i++){
				line=(to_string(goals.x[i])+" "+to_string(goals.y[i])+" "+to_string(t_gen[i])+" "+to_string(t_rx[i])+"\n");
				file<<line;
			}
			file.close();
		}else cout<<"No se puede abrir el fichero"<<endl;
	} // Guardar (se carga todo lo que había)

	void save_(const char* filename, Poss<int> goals, vector<float> t_gen, vector<float> t_rx){
		ofstream file(filename, ios::app); // abrir para escribir
		string line;
		if(file.is_open()){
			for(int i=0; i<goals.x.size(); i++){
				line=(to_string(goals.x[i])+" "+to_string(goals.y[i])+" "+to_string(t_gen[i])+" "+to_string(t_rx[i])+"\n");
				file<<line;
			}
			file.close();
		}else cout<<"No se puede abrir el fichero"<<endl;
	} // Guardar (Escribe detrás de lo que había)
	
	Poss<int> load(const char* filename){
		Poss<int> res;
		ifstream file(filename);
		string line;
		string number;
		int cont;
		if(file.is_open()){
			while(getline(file,line)){
				cont=0;
				for(int i=0; i<line.size(); i++){
					//cout<<number<<endl;
					//cin.get();
					if(line[i]==' ' && !cont){
						res.x.push_back(stoi(number));
						number.clear();
						cont++;
					}else if(line[i]==' ' && cont==1){
						res.y.push_back(stoi(number));
						number.clear();
						cont++;
					}else if(line[i]!=' '){
						number+=line[i];
					}
					if(cont==2){
						cont=0;
						line.clear();
						break;
					}
				}
			}
		}else cout<<"No se puede abrir el fichero"<<endl;
		return res;
	}


}


#endif